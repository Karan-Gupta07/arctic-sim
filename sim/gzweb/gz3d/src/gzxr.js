/*
 * Read-only WebXR presentation for the existing gzweb scene.
 *
 * gzweb already owns world loading and live pose updates. This adapter only
 * supplies stereo eye cameras to its renderer. It never advertises or
 * publishes a Gazebo topic, and headset motion remains local to the viewer.
 */
GZ3D.WebXRView = function(scene)
{
  this.scene = scene;
  this.renderer = scene.renderer;
  this.gl = this.renderer.getContext();
  this.session = null;
  this.layer = null;
  this.referenceSpace = null;
  this.baseViewerInverse = null;
  this.activeRole = null;
  this.activeAnchor = null;
  this.sensorMatrix = new THREE.Matrix4();
  this.eyeCameras = [];
  this.arrayCamera = new THREE.ArrayCamera(this.eyeCameras);
  this.arrayCamera.matrixAutoUpdate = false;
  this.centerWorld = new THREE.Matrix4();
  this.viewerMatrix = new THREE.Matrix4();
  this.deltaMatrix = new THREE.Matrix4();
  this.eyeWorld = new THREE.Matrix4();
  this.xrToGazebo = GZ3D.WebXRView.xrToGazeboMatrix();
  this.previousSize = null;
  this.previousPixelRatio = 1;
  this.framebufferWidth = 0;
  this.framebufferHeight = 0;
  this.availableRoles = [];
  this.hiddenRole = null;
  this.hiddenRoleWasVisible = true;
  this.supports = {vr: false, ar: false};
  this.sessionMode = null;
  this.viewMode = null;
  this.vehicleRole = null;
  this.mapScene = null;
  this.mapRoot = null;
  this.mapMarkers = [];
  this.mapScale = 0;
  this.mapPlaced = false;
  this.hitTestSource = null;
  this.previousClearColor = null;
  this.previousClearAlpha = 1;
  this.mapHelp = null;
  this.boatNotice = false;
  this.missionEventSeen = null;
  this.missionPolling = false;

  this.roles = [
    {
      name: 'quadcopter',
      pose: [0.20, 0, 0.65, 0, 0.20, 0],
      hideRoot: true
    },
    {
      name: 'fixed-wing',
      link: 'base_link',
      pose: [0.55, 0, 0.06, 0, 0.140, 0]
    },
    {
      name: 'tower-1',
      link: 'tilt_link',
      pose: [0.18, 0.038, 0, 0, 0, 0],
      hideLink: 'tilt_link'
    },
    {
      name: 'tower-2',
      link: 'tilt_link',
      pose: [0.18, 0.038, 0, 0, 0, 0],
      hideLink: 'tilt_link'
    },
    {
      name: 'rover',
      link: 'base_link',
      contains: 'rover_front_camera',
      pose: [0, 0, 0, 0, 0, 0]
    },
    {
      name: 'map-ar',
      label: 'Tabletop map (AR)',
      isMap: true
    }
  ];

  this._installRenderHooks();
  this._createControls();
  this._createHud();
  this._refreshRoles();

  var that = this;
  this.roleTimer = window.setInterval(function()
  {
    that._refreshRoles();
  }, 3000);
  this.missionTimer = window.setInterval(function()
  {
    that._pollMissionEvent();
  }, 1000);
  this._pollMissionEvent();
};

/*
 * WebXR is Y-up with -Z forward. Gazebo vehicles are Z-up with +X forward
 * and +Y left. This proper rotation maps XR right/up/back to
 * Gazebo right/up/back: +X -> -Y, +Y -> +Z, +Z -> -X.
 */
GZ3D.WebXRView.xrToGazeboMatrix = function()
{
  return new THREE.Matrix4().set(
      0, 0, -1, 0,
      -1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 0, 1);
};

GZ3D.WebXRView.findAnchor = function(root, role)
{
  var found = null;
  if (!root || !role.link)
  {
    return root;
  }
  root.traverse(function(object)
  {
    if (found || !object.name)
    {
      return;
    }
    var linkMatch = object.name === role.link ||
        object.name.slice(-(role.link.length + 2)) === '::' + role.link;
    var scopeMatch = !role.contains ||
        object.name.indexOf(role.contains) !== -1;
    if (linkMatch && scopeMatch)
    {
      found = object;
    }
  });
  return found || root;
};

GZ3D.WebXRView.prototype._restoreHiddenRole = function()
{
  if (!this.hiddenRole)
  {
    return;
  }
  this.hiddenRole.visible = this.hiddenRoleWasVisible;
  this.hiddenRole = null;
};

GZ3D.WebXRView.prototype._hideActiveRole = function()
{
  this._restoreHiddenRole();
  if (!this.session || !this.activeRole)
  {
    return;
  }
  var root = this.scene.getByName(this.activeRole.name);
  if (!root)
  {
    return;
  }
  var hidden = null;
  if (this.activeRole.hideRoot)
  {
    hidden = root;
  }
  else if (this.activeRole.hideLink)
  {
    hidden = GZ3D.WebXRView.findAnchor(root, {
      link: this.activeRole.hideLink,
      contains: this.activeRole.hideContains
    });
    if (hidden === root)
    {
      hidden = null;
    }
  }
  if (!hidden)
  {
    return;
  }
  this.hiddenRole = hidden;
  this.hiddenRoleWasVisible = hidden.visible;
  hidden.visible = false;
};

GZ3D.WebXRView.prototype._installRenderHooks = function()
{
  var that = this;
  this.desktopRender = this.scene.render.bind(this.scene);
  this.scene.render = function()
  {
    if (!that.session)
    {
      that.desktopRender();
    }
  };

  this.originalSetRenderTarget =
      this.renderer.setRenderTarget.bind(this.renderer);
  this.renderer.setRenderTarget = function(target)
  {
    that.originalSetRenderTarget(target);
    if (that.session && that.layer && !target)
    {
      that.gl.bindFramebuffer(
          that.gl.FRAMEBUFFER, that.layer.framebuffer);
    }
  };
};

GZ3D.WebXRView.prototype._createControls = function()
{
  var style = document.createElement('style');
  style.textContent =
      '#arctic-xr{position:fixed;left:50%;bottom:18px;z-index:100001;'+
      'transform:translateX(-50%);display:flex;align-items:center;gap:7px;'+
      'padding:8px 10px;border:1px solid #35505a;border-radius:6px;'+
      'background:rgba(10,17,20,.94);color:#dfe8ec;'+
      'font:12px sans-serif}'+
      '#arctic-xr select,#arctic-xr button{height:30px;border-radius:4px;'+
      'border:1px solid #3d8ba3;background:#12303a;color:#dfe8ec;'+
      'padding:0 10px;font:inherit}'+
      '#arctic-xr button:disabled{opacity:.45}'+
      '#arctic-xr-status{color:#8fa6ae;max-width:260px}';
  document.head.appendChild(style);

  this.controls = document.createElement('div');
  this.controls.id = 'arctic-xr';
  this.controls.innerHTML =
      '<strong>Immersive view</strong>'+
      '<select data-role="none" aria-label="Camera role"></select>'+
      '<button data-role="none" disabled>Enter VR</button>'+
      '<span id="arctic-xr-status">Checking WebXR...</span>';
  document.body.appendChild(this.controls);
  this.select = this.controls.querySelector('select');
  this.button = this.controls.querySelector('button');
  this.status = this.controls.querySelector('#arctic-xr-status');

  var that = this;
  this.select.addEventListener('change', function()
  {
    that._selectRole(that.select.value);
  });
  this.button.addEventListener('click', function()
  {
    if (that.session)
    {
      that.session.end();
    }
    else
    {
      that.start();
    }
  });

  if (!navigator.xr)
  {
    this._setStatus('WebXR needs Quest Browser on localhost or HTTPS.');
    return;
  }
  ['immersive-vr', 'immersive-ar'].forEach(function(mode)
  {
    navigator.xr.isSessionSupported(mode).then(function(supported)
    {
      that.supports[mode === 'immersive-ar' ? 'ar' : 'vr'] = supported;
      that._updateModeButton();
    }, function()
    {
      that._updateModeButton();
    });
  });
};

GZ3D.WebXRView.prototype._updateModeButton = function()
{
  if (this.session)
  {
    return;
  }
  var isMap = this.activeRole && this.activeRole.isMap;
  this.button.textContent = isMap ? 'Enter AR map' : 'Enter VR';
  this.button.disabled = !this.activeRole ||
      !(isMap ? this.supports.ar : this.supports.ar || this.supports.vr);
  this._setStatus(this.button.disabled ?
      (isMap ? 'Immersive AR is unavailable in this browser.' :
          'Immersive VR is unavailable in this browser.') :
      (isMap ? 'Point at a table, then use the left trigger to place the map.' :
          'Choose a role, then enter VR.'));
};

GZ3D.WebXRView.prototype._createHud = function()
{
  this.hudCanvas = document.createElement('canvas');
  this.hudCanvas.width = 768;
  this.hudCanvas.height = 160;
  this.hudTexture = new THREE.CanvasTexture(this.hudCanvas);
  this.hudMaterial = new THREE.MeshBasicMaterial({
    map: this.hudTexture,
    transparent: true,
    depthTest: false,
    depthWrite: false
  });
  this.hud = new THREE.Mesh(
      new THREE.PlaneGeometry(0.9, 0.1875), this.hudMaterial);
  this.hud.matrixAutoUpdate = false;
  this.hud.renderOrder = 100000;
  this.hud.visible = false;
  this.scene.scene.add(this.hud);
  this.hudOffset = new THREE.Matrix4().makeTranslation(0, -0.42, -1.1);
  this._drawHud('Waiting for a vehicle');
};

GZ3D.WebXRView.prototype._drawHud = function(title)
{
  var context = this.hudCanvas.getContext('2d');
  context.clearRect(0, 0, this.hudCanvas.width, this.hudCanvas.height);
  context.fillStyle = 'rgba(10, 17, 20, 0.88)';
  context.fillRect(0, 0, this.hudCanvas.width, this.hudCanvas.height);
  context.strokeStyle = '#4b9caf';
  context.lineWidth = 5;
  context.strokeRect(3, 3, this.hudCanvas.width - 6,
      this.hudCanvas.height - 6);
  context.fillStyle = this.boatNotice ? '#ffcc00' : '#dfe8ec';
  context.textAlign = 'center';
  context.font = '600 42px sans-serif';
  context.fillText(this.boatNotice ? 'BOAT DETECTED' :
      title + ' - READ ONLY',
      this.hudCanvas.width / 2, 68, this.hudCanvas.width - 30);
  context.fillStyle = '#8fdce9';
  context.font = '27px sans-serif';
  context.fillText(this.sessionMode === 'immersive-ar' ?
      'Right trigger: AR map | Left trigger: next view' :
      'Right trigger: next | Left trigger: previous',
      this.hudCanvas.width / 2, 116, this.hudCanvas.width - 30);
  this.hudTexture.needsUpdate = true;
};

GZ3D.WebXRView.prototype._setStatus = function(message)
{
  this.status.textContent = message;
};

GZ3D.WebXRView.prototype._pollMissionEvent = function()
{
  if (this.missionPolling || !window.fetch)
  {
    return;
  }
  this.missionPolling = true;
  var that = this;
  var url = window.location.protocol + '//' + window.location.hostname +
      ':8090/api/mission-event';
  window.fetch(url, {cache: 'no-store'}).then(function(response)
  {
    return response.json();
  }).then(function(data)
  {
    that.missionPolling = false;
    var when = Number(data.boat_detected_at) || 0;
    if (that.missionEventSeen !== null && when > that.missionEventSeen)
    {
      that._showBoatNotice();
    }
    that.missionEventSeen = when;
  }, function()
  {
    that.missionPolling = false;
  });
};

GZ3D.WebXRView.prototype._showBoatNotice = function()
{
  var that = this;
  var oldStatus = this.status.textContent;
  this.boatNotice = true;
  this._setStatus('Boat detected');
  this._drawHud(this.activeRole ? this.activeRole.name :
      'Waiting for a vehicle');
  this._drawMapHelp();
  window.setTimeout(function()
  {
    that.boatNotice = false;
    that._drawHud(that.activeRole ? that.activeRole.name :
        'Waiting for a vehicle');
    that._drawMapHelp();
    if (that.status.textContent === 'Boat detected')
    {
      that._setStatus(oldStatus);
    }
  }, 1000);
};

GZ3D.WebXRView.prototype._refreshRoles = function()
{
  var available = [];
  for (var i = 0; i < this.roles.length; ++i)
  {
    if (this.roles[i].isMap ? this.scene.heightmap :
        this.scene.getByName(this.roles[i].name))
    {
      available.push(this.roles[i]);
    }
  }
  var old = this.availableRoles.map(function(role)
  {
    return role.name;
  }).join('|');
  var next = available.map(function(role)
  {
    return role.name;
  }).join('|');
  this.availableRoles = available;
  if (old === next)
  {
    return;
  }

  while (this.select.firstChild)
  {
    this.select.removeChild(this.select.firstChild);
  }
  for (var j = 0; j < available.length; ++j)
  {
    var option = document.createElement('option');
    option.value = available[j].name;
    option.textContent = available[j].label || available[j].name;
    this.select.appendChild(option);
  }
  this.select.disabled = !!this.session || available.length === 0;

  var retained = this.activeRole && available.some(function(role)
  {
    return role.name === this.activeRole.name;
  }, this);
  if (retained)
  {
    this.select.value = this.activeRole.name;
  }
  else if (available.length)
  {
    this._selectRole(available[0].name);
  }
  else
  {
    this._restoreHiddenRole();
    this.activeRole = null;
    this.activeAnchor = null;
    this._setStatus('Waiting for simulated vehicles...');
    this._updateModeButton();
  }
};

GZ3D.WebXRView.prototype._poseMatrix = function(values)
{
  var position = new THREE.Vector3(values[0], values[1], values[2]);
  var rotation = new THREE.Euler(
      values[3], values[4], values[5], 'XYZ');
  var quaternion = new THREE.Quaternion().setFromEuler(rotation);
  return new THREE.Matrix4().compose(
      position, quaternion, new THREE.Vector3(1, 1, 1));
};

GZ3D.WebXRView.prototype._selectRole = function(name)
{
  for (var i = 0; i < this.availableRoles.length; ++i)
  {
    if (this.availableRoles[i].name === name)
    {
      if (this.session && !!this.availableRoles[i].isMap !==
          (this.viewMode === 'map'))
      {
        return;
      }
      this.activeRole = this.availableRoles[i];
      if (!this.activeRole.isMap)
      {
        this.vehicleRole = this.activeRole.name;
      }
      var root = this.scene.getByName(this.activeRole.name);
      this.activeAnchor = this.activeRole.isMap ? null :
          GZ3D.WebXRView.findAnchor(root, this.activeRole);
      if (!this.activeRole.isMap)
      {
        this.sensorMatrix.copy(this._poseMatrix(this.activeRole.pose));
      }
      this.baseViewerInverse = null;
      this.select.value = name;
      if (!this.activeRole.isMap)
      {
        this._drawHud(name);
      }
      this._updateModeButton();
      this._hideActiveRole();
      return;
    }
  }
};

GZ3D.WebXRView.prototype._cycleRole = function(step)
{
  var roles = this.availableRoles.filter(function(role)
  {
    return !role.isMap;
  });
  if (!roles.length)
  {
    return;
  }
  var index = 0;
  for (var i = 0; i < roles.length; ++i)
  {
    if (this.activeRole &&
        roles[i].name === this.activeRole.name)
    {
      index = i;
      break;
    }
  }
  index = (index + step + roles.length) % roles.length;
  this._selectRole(roles[index].name);
};

GZ3D.WebXRView.prototype.start = function()
{
  if (!navigator.xr || !this.activeRole || this.session)
  {
    return;
  }
  var that = this;
  var isMap = !!this.activeRole.isMap;
  this.button.disabled = true;
  this._setStatus(isMap ? 'Starting tabletop map...' :
      'Starting immersive view...');
  var mode = this.supports.ar ? 'immersive-ar' : 'immersive-vr';
  navigator.xr.requestSession(mode, {
    requiredFeatures: ['local-floor'],
    optionalFeatures: this.supports.ar ? ['hit-test'] : []
  }).then(function(session)
  {
    that._beginSession(session);
  }, function(error)
  {
    that.button.disabled = false;
    that._setStatus('Could not enter ' + (isMap ? 'AR map' : 'VR') +
        ': ' + error.message);
  });
};

GZ3D.WebXRView.prototype._beginSession = function(session)
{
  var that = this;
  this.session = session;
  this.sessionMode = this.supports.ar ? 'immersive-ar' : 'immersive-vr';
  this.viewMode = this.activeRole.isMap ? 'map' : 'vehicle';
  session.addEventListener('end', function()
  {
    that._endSession();
  });
  this.gl.makeXRCompatible().then(function()
  {
    that.layer = new window.XRWebGLLayer(session, that.gl, {
      alpha: that.sessionMode === 'immersive-ar',
      antialias: true
    });
    session.updateRenderState({
      baseLayer: that.layer,
      depthNear: 0.05,
      depthFar: 50000
    });
    return session.requestReferenceSpace('local-floor');
  }).then(function(referenceSpace)
  {
    that.referenceSpace = referenceSpace;
    that.previousSize = that.renderer.getSize();
    that.previousPixelRatio = that.renderer.getPixelRatio();
    that._resizeForLayer();
    that.previousClearColor = that.renderer.getClearColor().clone();
    that.previousClearAlpha = that.renderer.getClearAlpha();
    if (that.sessionMode === 'immersive-ar')
    {
      if (that.scene.heightmap)
      {
        that._prepareMap();
      }
      if (session.requestHitTestSource)
      {
        session.requestReferenceSpace('viewer').then(function(space)
        {
          return session.requestHitTestSource({space: space});
        }).then(function(source)
        {
          if (that.session === session)
          {
            that.hitTestSource = source;
          }
          else
          {
            source.cancel();
          }
        }, function() {});
      }
    }
    if (that.viewMode === 'vehicle')
    {
      that._hideActiveRole();
      that.hud.visible = true;
      that._drawHud(that.activeRole.name);
    }
    that._setViewClearColor();
    that.select.disabled = true;
    that.button.textContent = 'Exit immersive view';
    that.button.disabled = false;
    that._setStatus(that.viewMode === 'map' ?
        'Left trigger places or moves the map. Right trigger opens VR.' :
        'Immersive read-only view active.');
    session.addEventListener('select', function(event)
    {
      if (that.sessionMode === 'immersive-ar' &&
          event.inputSource.handedness === 'right')
      {
        if (!that.mapScene && that.scene.heightmap)
        {
          that._prepareMap();
        }
        if (!that.mapScene)
        {
          that._setStatus('Terrain is still loading.');
          return;
        }
        if (that.viewMode === 'map')
        {
          var hit = that._mapHit(event.frame, event.inputSource);
          if (hit === 'target_vessel')
          {
            return;
          }
          if (hit)
          {
            that.vehicleRole = hit;
          }
        }
        that._switchView();
      }
      else if (that.viewMode === 'map')
      {
        that.mapPlaced = !that.mapPlaced;
        that._setStatus(that.mapPlaced ? 'Tabletop map placed.' :
            'Move the map by looking at a new spot, then press left trigger.');
      }
      else
      {
        that._cycleRole(event.inputSource.handedness === 'left' ? -1 : 1);
      }
    });
    session.requestAnimationFrame(function(time, frame)
    {
      that._onXRFrame(time, frame);
    });
  }, function(error)
  {
    that._setStatus('WebXR setup failed: ' + error.message);
    session.end();
  });
};

GZ3D.WebXRView.prototype._setViewClearColor = function()
{
  this.renderer.setClearColor(this.viewMode === 'map' ? 0x000000 :
      this.previousClearColor, this.viewMode === 'map' ? 0 : 1);
};

GZ3D.WebXRView.prototype._switchView = function()
{
  if (this.viewMode === 'map')
  {
    var roles = this.availableRoles.filter(function(item)
    {
      return !item.isMap;
    });
    if (!roles.length)
    {
      this._setStatus('Waiting for a simulated asset viewpoint.');
      return;
    }
    var role = this.vehicleRole || roles[0].name;
    this.viewMode = 'vehicle';
    this._selectRole(role);
    this.hud.visible = true;
    this._setStatus('Vehicle view. Right trigger returns to the AR map.');
  }
  else
  {
    this._restoreHiddenRole();
    this.viewMode = 'map';
    this._selectRole('map-ar');
    this.hud.visible = false;
    this._setStatus('AR map. Left trigger places or moves it.');
  }
  this._setViewClearColor();
};

GZ3D.WebXRView.prototype._mapHit = function(frame, inputSource)
{
  if (!inputSource || !inputSource.targetRaySpace)
  {
    return null;
  }
  var pose = frame.getPose(inputSource.targetRaySpace, this.referenceSpace);
  if (!pose)
  {
    return null;
  }
  var matrix = new THREE.Matrix4().fromArray(pose.transform.matrix);
  var origin = new THREE.Vector3().setFromMatrixPosition(matrix);
  var direction = new THREE.Vector3(0, 0, -1).transformDirection(matrix);
  var ray = new THREE.Raycaster(origin, direction, 0, 5);
  var closest = null;
  var distance = Infinity;
  this.mapScene.updateMatrixWorld(true);
  for (var i = 0; i < this.mapMarkers.length; ++i)
  {
    var marker = this.mapMarkers[i];
    if (!marker.object.visible)
    {
      continue;
    }
    var hits = ray.intersectObject(marker.hit, false);
    if (hits.length && hits[0].distance < distance)
    {
      closest = marker.name;
      distance = hits[0].distance;
    }
  }
  return closest;
};

GZ3D.WebXRView.prototype._resizeForLayer = function()
{
  if (!this.layer)
  {
    return;
  }
  if (this.framebufferWidth === this.layer.framebufferWidth &&
      this.framebufferHeight === this.layer.framebufferHeight)
  {
    return;
  }
  this.framebufferWidth = this.layer.framebufferWidth;
  this.framebufferHeight = this.layer.framebufferHeight;
  this.renderer.setDrawingBufferSize(
      this.framebufferWidth, this.framebufferHeight, 1);
};

GZ3D.WebXRView.prototype._drawMapHelp = function()
{
  if (!this.mapHelpCanvas)
  {
    return;
  }
  var context = this.mapHelpCanvas.getContext('2d');
  context.clearRect(0, 0, 1024, 128);
  context.fillStyle = 'rgba(10,17,20,0.88)';
  context.fillRect(0, 0, 1024, 128);
  context.fillStyle = this.boatNotice ? '#ffcc00' : '#dfe8ec';
  context.textAlign = 'center';
  context.font = this.boatNotice ? 'bold 52px sans-serif' :
      '38px sans-serif';
  context.fillText(this.boatNotice ? 'BOAT DETECTED' :
      'LEFT: place / move    RIGHT: camera pins / VR', 512, 79, 990);
  this.mapHelpTexture.needsUpdate = true;
};

/* Reuse gzweb's loaded terrain mesh; markers read the same live model poses. */
GZ3D.WebXRView.prototype._prepareMap = function()
{
  if (!this.scene.heightmap)
  {
    throw new Error('Terrain is still loading');
  }
  this.mapScene = new THREE.Scene();
  this.mapRoot = new THREE.Group();
  this.mapScene.add(this.mapRoot);
  this.mapHelpCanvas = document.createElement('canvas');
  this.mapHelpCanvas.width = 1024;
  this.mapHelpCanvas.height = 128;
  this.mapHelpTexture = new THREE.CanvasTexture(this.mapHelpCanvas);
  this.mapHelp = new THREE.Mesh(
      new THREE.PlaneGeometry(1.1, 0.138),
      new THREE.MeshBasicMaterial({
        map: this.mapHelpTexture, transparent: true,
        depthTest: false, depthWrite: false
      }));
  this.mapHelp.matrixAutoUpdate = false;
  this.mapHelp.renderOrder = 100000;
  this.mapHelpOffset = new THREE.Matrix4().makeTranslation(0, -0.42, -1.1);
  this.mapScene.add(this.mapHelp);
  this._drawMapHelp();
  this.mapRays = [];
  for (var r = 0; r < 2; ++r)
  {
    var geometry = new THREE.Geometry();
    geometry.vertices.push(new THREE.Vector3(), new THREE.Vector3(0, 0, -2));
    var line = new THREE.Line(geometry,
        new THREE.LineBasicMaterial({color: r ? 0xffcc00 : 0x5bc8e0}));
    line.visible = false;
    this.mapScene.add(line);
    this.mapRays.push(line);
  }
  this.mapPlaced = false;
  var extent = 6500;
  this.scene.heightmap.traverse(function(object)
  {
    if (object.geometry && object.geometry.parameters &&
        object.geometry.parameters.width)
    {
      extent = object.geometry.parameters.width;
    }
  });
  this.mapScale = 1.3 / extent;

  var terrain = this.scene.heightmap.clone(true);
  terrain.rotation.x = -Math.PI / 2;
  terrain.scale.set(this.mapScale, this.mapScale, this.mapScale * 3);
  this.mapRoot.add(terrain);
  var base = new THREE.Mesh(new THREE.PlaneGeometry(1.32, 1.32),
      new THREE.MeshBasicMaterial({
        color: 0x10232d, transparent: true, opacity: 0.8,
        side: THREE.DoubleSide
      }));
  base.rotation.x = -Math.PI / 2;
  base.position.y = -0.015;
  this.mapRoot.add(base);

  var roles = [
    ['quadcopter', 0x57d38c], ['fixed-wing', 0x57d38c],
    ['tower-1', 0x5bc8e0], ['tower-2', 0x5bc8e0],
    ['target_vessel', 0xffcc00]
  ];
  this.mapMarkers = [];
  for (var i = 0; i < roles.length; ++i)
  {
    var marker = new THREE.Group();
    var stem = new THREE.Mesh(
        new THREE.CylinderGeometry(0.003, 0.003, 0.12, 6),
        new THREE.MeshBasicMaterial({color: roles[i][1]}));
    stem.position.y = 0.06;
    marker.add(stem);
    var dot = new THREE.Mesh(new THREE.SphereGeometry(0.023, 10, 8),
        new THREE.MeshBasicMaterial({color: roles[i][1]}));
    dot.position.y = 0.12;
    marker.add(dot);
    var canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 64;
    var ctx = canvas.getContext('2d');
    ctx.fillStyle = 'rgba(10,17,20,0.9)';
    ctx.fillRect(0, 0, 256, 64);
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.font = 'bold 27px sans-serif';
    ctx.fillText(roles[i][0].replace('target_', ''), 128, 43, 245);
    var label = new THREE.Mesh(new THREE.PlaneGeometry(0.26, 0.065),
        new THREE.MeshBasicMaterial({
      map: new THREE.CanvasTexture(canvas), transparent: true,
      depthTest: false, depthWrite: false, side: THREE.DoubleSide
    }));
    label.position.y = 0.16;
    marker.add(label);
    var hit = new THREE.Mesh(new THREE.SphereGeometry(0.075, 8, 6),
        new THREE.MeshBasicMaterial({transparent: true, opacity: 0,
          depthWrite: false}));
    hit.position.y = 0.12;
    marker.add(hit);
    marker.visible = false;
    this.mapRoot.add(marker);
    this.mapMarkers.push({name: roles[i][0], object: marker,
      label: label, hit: hit});
  }
};

GZ3D.WebXRView.mapPosition = function(world, scale)
{
  return new THREE.Vector3(world.x * scale,
      world.z * scale * 3, -world.y * scale);
};

GZ3D.WebXRView.prototype._updateMapMarkers = function()
{
  var world = new THREE.Vector3();
  for (var i = 0; i < this.mapMarkers.length; ++i)
  {
    var marker = this.mapMarkers[i];
    var model = this.scene.getByName(marker.name);
    marker.object.visible = !!model;
    if (model)
    {
      model.getWorldPosition(world);
      marker.object.position.copy(
          GZ3D.WebXRView.mapPosition(world, this.mapScale));
    }
  }
};

GZ3D.WebXRView.prototype._ensureEyeCameras = function(count)
{
  while (this.eyeCameras.length < count)
  {
    var camera = new THREE.PerspectiveCamera();
    camera.matrixAutoUpdate = false;
    camera.bounds = new THREE.Vector4();
    this.eyeCameras.push(camera);
  }
  this.eyeCameras.length = count;
  this.arrayCamera.cameras = this.eyeCameras;
};

GZ3D.WebXRView.prototype._applyView = function(camera, view, viewport,
    anchorWorld)
{
  this.viewerMatrix.fromArray(view.transform.matrix);
  this.deltaMatrix.multiplyMatrices(
      this.baseViewerInverse, this.viewerMatrix);
  this.eyeWorld.copy(anchorWorld)
      .multiply(this.sensorMatrix)
      .multiply(this.xrToGazebo)
      .multiply(this.deltaMatrix);
  camera.matrix.copy(this.eyeWorld);
  camera.matrixWorld.copy(this.eyeWorld);
  camera.matrixWorldInverse.getInverse(this.eyeWorld);
  camera.projectionMatrix.fromArray(view.projectionMatrix);
  camera.bounds.set(
      viewport.x / this.layer.framebufferWidth,
      viewport.y / this.layer.framebufferHeight,
      viewport.width / this.layer.framebufferWidth,
      viewport.height / this.layer.framebufferHeight);
};

GZ3D.WebXRView.prototype._onARFrame = function(frame, pose)
{
  this._resizeForLayer();
  this._ensureEyeCameras(pose.views.length);
  this.viewerMatrix.fromArray(pose.transform.matrix);
  if (!this.mapPlaced)
  {
    var position = null;
    if (this.hitTestSource && frame.getHitTestResults)
    {
      var hits = frame.getHitTestResults(this.hitTestSource);
      if (hits.length)
      {
        var hitPose = hits[0].getPose(this.referenceSpace);
        if (hitPose)
        {
          position = hitPose.transform.position;
        }
      }
    }
    if (position)
    {
      this.mapRoot.position.set(position.x, position.y + 0.02, position.z);
    }
    else
    {
      this.mapRoot.position.set(0, -0.55, -1.2)
          .applyMatrix4(this.viewerMatrix);
    }
  }

  this.scene.scene.updateMatrixWorld(true);
  this._updateMapMarkers();
  this.mapScene.updateMatrixWorld(true);
  var head = new THREE.Vector3().setFromMatrixPosition(this.viewerMatrix);
  for (var m = 0; m < this.mapMarkers.length; ++m)
  {
    var marker = this.mapMarkers[m];
    if (marker.object.visible)
    {
      marker.label.lookAt(marker.object.worldToLocal(head.clone()));
    }
  }
  this.mapHelp.matrix.copy(this.viewerMatrix).multiply(this.mapHelpOffset);
  this.mapHelp.matrixWorldNeedsUpdate = true;
  for (var rayIndex = 0; rayIndex < this.mapRays.length; ++rayIndex)
  {
    var source = this.session.inputSources[rayIndex];
    var rayPose = source && source.targetRaySpace &&
        frame.getPose(source.targetRaySpace, this.referenceSpace);
    var line = this.mapRays[rayIndex];
    line.visible = !!rayPose;
    if (rayPose)
    {
      line.matrix.fromArray(rayPose.transform.matrix);
      line.matrixAutoUpdate = false;
      line.matrixWorldNeedsUpdate = true;
    }
  }
  for (var i = 0; i < pose.views.length; ++i)
  {
    var view = pose.views[i];
    var camera = this.eyeCameras[i];
    var viewport = this.layer.getViewport(view);
    camera.matrix.fromArray(view.transform.matrix);
    camera.matrixWorld.copy(camera.matrix);
    camera.matrixWorldInverse.getInverse(camera.matrixWorld);
    camera.projectionMatrix.fromArray(view.projectionMatrix);
    camera.bounds.set(
        viewport.x / this.layer.framebufferWidth,
        viewport.y / this.layer.framebufferHeight,
        viewport.width / this.layer.framebufferWidth,
        viewport.height / this.layer.framebufferHeight);
  }
  this.arrayCamera.matrix.fromArray(pose.transform.matrix);
  this.arrayCamera.matrixWorld.copy(this.arrayCamera.matrix);
  this.arrayCamera.matrixWorldInverse.getInverse(this.arrayCamera.matrixWorld);
  this.arrayCamera.projectionMatrix.copy(
      this.eyeCameras[0].projectionMatrix);
  this.renderer.setRenderTarget(null);
  this.renderer.setScissorTest(false);
  this.renderer.clear(true, true, true);
  this.renderer.render(this.mapScene, this.arrayCamera);
};

GZ3D.WebXRView.prototype._onXRFrame = function(time, frame)
{
  if (!this.session || frame.session !== this.session)
  {
    return;
  }
  var that = this;
  this.session.requestAnimationFrame(function(nextTime, nextFrame)
  {
    that._onXRFrame(nextTime, nextFrame);
  });
  var pose = frame.getViewerPose(this.referenceSpace);
  if (!pose || !this.activeRole)
  {
    return;
  }
  if (this.viewMode === 'map')
  {
    this._onARFrame(frame, pose);
    return;
  }
  var root = this.scene.getByName(this.activeRole.name);
  if (!root)
  {
    return;
  }
  if (!this.activeAnchor || this.activeAnchor === root)
  {
    this.activeAnchor = GZ3D.WebXRView.findAnchor(root, this.activeRole);
  }

  this.scene.scene.updateMatrixWorld(true);
  if (!this.baseViewerInverse)
  {
    this.baseViewerInverse = new THREE.Matrix4()
        .fromArray(pose.transform.matrix);
    this.baseViewerInverse.getInverse(this.baseViewerInverse);
  }
  this._resizeForLayer();
  this._ensureEyeCameras(pose.views.length);

  var anchorWorld = this.activeAnchor.matrixWorld;
  for (var i = 0; i < pose.views.length; ++i)
  {
    var viewport = this.layer.getViewport(pose.views[i]);
    this._applyView(this.eyeCameras[i], pose.views[i], viewport,
        anchorWorld);
  }

  this.viewerMatrix.fromArray(pose.transform.matrix);
  this.deltaMatrix.multiplyMatrices(
      this.baseViewerInverse, this.viewerMatrix);
  this.centerWorld.copy(anchorWorld)
      .multiply(this.sensorMatrix)
      .multiply(this.xrToGazebo)
      .multiply(this.deltaMatrix);
  this.arrayCamera.matrix.copy(this.centerWorld);
  this.arrayCamera.matrixWorld.copy(this.centerWorld);
  this.arrayCamera.matrixWorldInverse.getInverse(this.centerWorld);
  this.arrayCamera.projectionMatrix.copy(
      this.eyeCameras[0].projectionMatrix);

  this.hud.matrix.copy(this.centerWorld).multiply(this.hudOffset);
  this.hud.matrixWorldNeedsUpdate = true;
  this.renderer.setRenderTarget(null);
  this.renderer.setScissorTest(false);
  this.renderer.clear(true, true, true);
  this.renderer.render(this.scene.scene, this.arrayCamera);
};

GZ3D.WebXRView.prototype._endSession = function()
{
  this._restoreHiddenRole();
  if (this.hitTestSource)
  {
    this.hitTestSource.cancel();
    this.hitTestSource = null;
  }
  if (this.previousClearColor)
  {
    this.renderer.setClearColor(
        this.previousClearColor, this.previousClearAlpha);
    this.previousClearColor = null;
  }
  this.session = null;
  this.sessionMode = null;
  this.viewMode = null;
  this.referenceSpace = null;
  this.layer = null;
  this.baseViewerInverse = null;
  this.hud.visible = false;
  this.gl.bindFramebuffer(this.gl.FRAMEBUFFER, null);
  if (this.previousSize)
  {
    this.renderer.setDrawingBufferSize(
        this.previousSize.width,
        this.previousSize.height,
        this.previousPixelRatio);
  }
  this.framebufferWidth = 0;
  this.framebufferHeight = 0;
  this.select.disabled = this.availableRoles.length === 0;
  this._updateModeButton();
  this.scene.setSize(window.innerWidth, window.innerHeight);
  this._setStatus('Exited immersive view.');
};
