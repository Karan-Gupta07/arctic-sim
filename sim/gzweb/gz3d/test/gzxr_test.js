describe('WebXR view adapter', function() {

  it('maps WebXR axes into the Gazebo vehicle frame', function() {
    var matrix = GZ3D.WebXRView.xrToGazeboMatrix();
    var right = new THREE.Vector3(1, 0, 0).applyMatrix4(matrix);
    var up = new THREE.Vector3(0, 1, 0).applyMatrix4(matrix);
    var forward = new THREE.Vector3(0, 0, -1).applyMatrix4(matrix);

    expect(right).toEqual(new THREE.Vector3(0, -1, 0));
    expect(up).toEqual(new THREE.Vector3(0, 0, 1));
    expect(forward).toEqual(new THREE.Vector3(1, 0, 0));
  });

  it('places live map markers over the matching terrain coordinates',
      function() {
    var p = GZ3D.WebXRView.mapPosition(
        new THREE.Vector3(100, 200, 30), 0.001);
    expect(p.x).toBeCloseTo(0.1, 6);
    expect(p.y).toBeCloseTo(0.09, 6);
    expect(p.z).toBeCloseTo(-0.2, 6);
  });

  it('selects a visible map pin with a controller ray', function() {
    var view = Object.create(GZ3D.WebXRView.prototype);
    view.referenceSpace = {};
    view.mapScene = new THREE.Scene();
    var marker = new THREE.Group();
    marker.position.z = -1;
    var hit = new THREE.Mesh(new THREE.SphereGeometry(0.075));
    marker.add(hit);
    view.mapScene.add(marker);
    view.mapMarkers = [{name: 'quadcopter', object: marker, hit: hit}];
    var frame = {getPose: function() {
      return {transform: {matrix: new THREE.Matrix4().toArray()}};
    }};

    expect(view._mapHit(frame, {targetRaySpace: {}})).toBe('quadcopter');
    marker.visible = false;
    expect(view._mapHit(frame, {targetRaySpace: {}})).toBe(null);
  });

  it('finds the scoped camera link without selecting a sibling', function() {
    var root = new THREE.Object3D();
    var sibling = new THREE.Object3D();
    var camera = new THREE.Object3D();
    sibling.name = 'other_model::tilt_link';
    camera.name = 'gimbal_small_2d::tilt_link';
    root.add(sibling);
    root.add(camera);

    var found = GZ3D.WebXRView.findAnchor(root, {
      link: 'tilt_link',
      contains: 'gimbal_small_2d'
    });

    expect(found).toBe(camera);
  });

  it('falls back to the role root while nested links are loading', function() {
    var root = new THREE.Object3D();
    expect(GZ3D.WebXRView.findAnchor(root, {
      link: 'missing_link'
    })).toBe(root);
  });

  it('aims the calibrated quadcopter view forward and slightly down',
      function() {
    var view = Object.create(GZ3D.WebXRView.prototype);
    var pose = view._poseMatrix([0.20, 0, 0.65, 0, 0.20, 0]);
    var basis = GZ3D.WebXRView.xrToGazeboMatrix();
    var matrix = pose.multiply(basis);
    var origin = new THREE.Vector3(0, 0, 0).applyMatrix4(matrix);
    var forward = new THREE.Vector3(0, 0, -1)
        .applyMatrix4(matrix).sub(origin).normalize();

    expect(origin.z).toBeCloseTo(0.65, 6);
    expect(forward.x).toBeGreaterThan(0);
    expect(forward.z).toBeLessThan(0);
  });

  it('hides only a tower camera head and restores its visibility', function() {
    var root = new THREE.Object3D();
    var mast = new THREE.Object3D();
    var head = new THREE.Object3D();
    mast.name = 'mast';
    head.name = 'tower-1::tilt_link';
    root.add(mast);
    root.add(head);
    var view = Object.create(GZ3D.WebXRView.prototype);
    view.session = {};
    view.activeRole = {name: 'tower-1', hideLink: 'tilt_link'};
    view.scene = {getByName: function() { return root; }};
    view.hiddenRole = null;
    view.hiddenRoleWasVisible = true;

    view._hideActiveRole();
    expect(root.visible).toBe(true);
    expect(mast.visible).toBe(true);
    expect(head.visible).toBe(false);
    view._restoreHiddenRole();
    expect(head.visible).toBe(true);
  });
});
