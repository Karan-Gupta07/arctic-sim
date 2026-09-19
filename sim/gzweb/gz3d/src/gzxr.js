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
  navigator.xr.isSessionSupported('immersive-vr').then(function(supported)
  {
    that.button.disabled = !supported;
    that._setStatus(supported ? 'Choose a role, then enter VR.' :
        'Immersive VR is unavailable in this browser.');
  }, function()
  {
    that._setStatus('Could not check WebXR support.');
  });
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
  context.fillStyle = '#dfe8ec';
  context.textAlign = 'center';
  context.font = '600 42px sans-serif';
  context.fillText(title + ' - READ ONLY',
      this.hudCanvas.width / 2, 68, this.hudCanvas.width - 30);
  context.fillStyle = '#8fdce9';
  context.font = '27px sans-serif';
  context.fillText('Right trigger: next | Left trigger: previous',
      this.hudCanvas.width / 2, 116, this.hudCanvas.width - 30);
  this.hudTexture.needsUpdate = true;
};

GZ3D.WebXRView.prototype._setStatus = function(message)
{
  this.status.textContent = message;
};

GZ3D.WebXRView.prototype._refreshRoles = function()
{
  var available = [];
  for (var i = 0; i < this.roles.length; ++i)
  {
    if (this.scene.getByName(this.roles[i].name))
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
    option.textContent = available[j].name;
    this.select.appendChild(option);
  }
  this.select.disabled = available.length === 0;

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
      this.activeRole = this.availableRoles[i];
      var root = this.scene.getByName(this.activeRole.name);
      this.activeAnchor = GZ3D.WebXRView.findAnchor(
          root, this.activeRole);
      this.sensorMatrix.copy(this._poseMatrix(this.activeRole.pose));
      this.baseViewerInverse = null;
      this.select.value = name;
      this._drawHud(name);
      this._setStatus('Selected ' + name + ' (read only).');
      this._hideActiveRole();
      return;
    }
  }
};

GZ3D.WebXRView.prototype._cycleRole = function(step)
{
  if (!this.availableRoles.length)
  {
    return;
  }
  var index = 0;
  for (var i = 0; i < this.availableRoles.length; ++i)
  {
    if (this.activeRole &&
        this.availableRoles[i].name === this.activeRole.name)
    {
      index = i;
      break;
    }
  }
  index = (index + step + this.availableRoles.length) %
      this.availableRoles.length;
  this._selectRole(this.availableRoles[index].name);
};

GZ3D.WebXRView.prototype.start = function()
{
  if (!navigator.xr || !this.activeRole || this.session)
  {
    return;
  }
  var that = this;
  this.button.disabled = true;
  this._setStatus('Starting immersive view...');
  navigator.xr.requestSession('immersive-vr', {
    requiredFeatures: ['local-floor']
  }).then(function(session)
  {
    that._beginSession(session);
  }, function(error)
  {
    that.button.disabled = false;
    that._setStatus('Could not enter VR: ' + error.message);
  });
};

GZ3D.WebXRView.prototype._beginSession = function(session)
{
  var that = this;
  this.session = session;
  session.addEventListener('end', function()
  {
    that._endSession();
  });
  this.gl.makeXRCompatible().then(function()
  {
    that.layer = new window.XRWebGLLayer(session, that.gl, {
      alpha: false,
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
    that._hideActiveRole();
    that.hud.visible = true;
    that.button.textContent = 'Exit VR';
    that.button.disabled = false;
    that._setStatus('Immersive read-only view active.');
    session.addEventListener('select', function(event)
    {
      that._cycleRole(event.inputSource.handedness === 'left' ? -1 : 1);
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
  this.session = null;
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
  this.button.textContent = 'Enter VR';
  this.button.disabled = false;
  this.scene.setSize(window.innerWidth, window.innerHeight);
  this._setStatus('Exited immersive view.');
};
