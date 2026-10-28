import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import { rad, deg, sdkTarget, relativeAngle, withinLimits, freshSnapshot, gripperGap } from './mapping.js?v=lerobot-3';

const $ = id => document.getElementById(id);
let meta, robot, reference = null, latest = null, receivedAt = 0, mode = 'live', selected = 0;
let errors = [], disconnected = true, pollInFlight = false, poseValues = Array(6).fill(0);
const qReference = Array(6).fill(0), signs = Array(6).fill(1), axisGroups = [], comGroups = [];
const rowElements = [], materialRecords = [];
let gapState = {requested: 0, applied: 0};
const viewport = $('viewport');
const scene = new THREE.Scene(); scene.background = new THREE.Color('#eaf0f4');
const camera = new THREE.PerspectiveCamera(39, 1, 0.002, 30); camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.setClearColor('#eaf0f4');
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.prepend(renderer.domElement);
renderer.domElement.setAttribute('aria-label', '3D Maker Arm model');
const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true; controls.dampingFactor = 0.1;
controls.minDistance = .10; controls.maxDistance = 4;
scene.add(new THREE.HemisphereLight(0xffffff, 0x768e9b, 2.4));
const key = new THREE.DirectionalLight(0xffffff, 3.2); key.position.set(.6, -.9, 1.4); scene.add(key);
const fill = new THREE.DirectionalLight(0xb9dbe5, 1.5); fill.position.set(-1, .8, .5); scene.add(fill);
const grid = new THREE.GridHelper(2, 40, 0x9cb0bd, 0xcbd7df); grid.rotation.x = Math.PI / 2; scene.add(grid);
const worldAxes = new THREE.AxesHelper(.075); scene.add(worldAxes);
const wrapper = new THREE.Group(); scene.add(wrapper);
const colors = [0x657b8b, 0x598b9c, 0x87a7b5, 0x6e909f, 0x719b9c, 0x7e9aaa, 0x667f8d];
const decorativeLoopMeshes = new Set(['meshes/part_046_solid_046.stl', 'meshes/part_047_solid_047.stl']);
const resize = () => { const w = viewport.clientWidth, h = viewport.clientHeight; renderer.setSize(w, h); camera.aspect = w/h; camera.updateProjectionMatrix(); };
new ResizeObserver(resize).observe(viewport);

function labelSprite(text, color = '#087e80') {
  const canvas = document.createElement('canvas'); canvas.width = 96; canvas.height = 96;
  const ctx = canvas.getContext('2d'); ctx.fillStyle = '#ffffff'; ctx.beginPath(); ctx.arc(48,48,36,0,Math.PI*2); ctx.fill();
  ctx.strokeStyle = color; ctx.lineWidth = 5; ctx.stroke(); ctx.fillStyle = color; ctx.font = 'bold 42px sans-serif'; ctx.textAlign='center'; ctx.textBaseline='middle';ctx.fillText(text,48,49);
  const texture = new THREE.CanvasTexture(canvas); texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({map:texture,depthTest:false,transparent:true})); sprite.scale.set(.033,.033,.033); sprite.renderOrder = 100; return sprite;
}

function updateDisplay() {
  const collision = $('collision').checked;
  for (const r of materialRecords) {
    if (r.collision) { r.mesh.visible = collision; }
    else { r.mesh.visible = !collision && !($('hide-loops').checked && r.decorativeLoop); r.mesh.material.color.setHex(r.linkIndex === selected + 1 ? 0x238d94 : colors[r.linkIndex % colors.length]); }
  }
  axisGroups.forEach((a,i) => { a.visible = $('axes').checked; a.scale.setScalar(i === selected ? 1.22 : 1); });
  comGroups.forEach(g => g.visible = $('com').checked);
}

function orientModel() {
  wrapper.rotation.set($('up').value === 'y' ? Math.PI/2 : 0, 0, 0);
  wrapper.position.set(0,0,0); wrapper.updateMatrixWorld(true);
  // Put only the base's visual bottom on the display grid; this is a view transform.
  const baseBox = new THREE.Box3();
  robot.links[meta.root].children.filter(c=>c.isURDFVisual).forEach(c=>baseBox.union(new THREE.Box3().setFromObject(c)));
  if (!baseBox.isEmpty()) wrapper.position.z = -baseBox.min.z;
  wrapper.updateMatrixWorld(true);
  fitView();
}

function visibleBounds() {
  const box = new THREE.Box3();
  for (const r of materialRecords) if (!r.collision) box.union(new THREE.Box3().setFromObject(r.mesh));
  return box;
}
function fitView(view='iso') {
  if (!robot) return;
  wrapper.updateMatrixWorld(true);
  const box = visibleBounds(), center=box.getCenter(new THREE.Vector3()), size=box.getSize(new THREE.Vector3());
  const distance=Math.max(size.length(),.4) / (2*Math.tan(THREE.MathUtils.degToRad(camera.fov/2))) * 1.12 / Math.min(camera.aspect,1);
  const directions={iso:[1.1,-1.8,1.0],front:[0,-1,.06],side:[1,0,.06],top:[0,-.001,1]};
  camera.position.copy(center).add(new THREE.Vector3(...directions[view]).normalize().multiplyScalar(distance));
  controls.target.copy(center); controls.update();
}

function selectJoint(i) {
  selected=i; $('selected').value=String(i); $('reference-angle').value=deg(qReference[i]||0);
  $('reference-angle').disabled=i===6; $('flip').disabled=i===6;
  $('reference-value').textContent=i===6?'Use jaw gap below':deg(qReference[i]).toFixed(1)+'°'; $('flip').checked=signs[i]<0;
  rowElements.forEach((r,k)=>r.row.classList.toggle('selected',k===i));
  const j=i===6?meta.all_joints.find(j=>j.name==='gripper_left_joint'):meta.joints[i], xyz=(j.origin.xyz||'0 0 0').split(/\s+/).map(Number);
  $('joint-detail').replaceChildren();
  const name=document.createElement('p');name.className='mono small';name.textContent=j.name; $('joint-detail').append(name);
  const dl=document.createElement('div');dl.className='dimension-list';
  for(const [label,value] of [['Parent → child',`${j.parent} → ${j.child}`],['Axis in joint frame',j.axis],['Origin offset in parent',xyz.map(x=>(x*1000).toFixed(1)).join(', ')+' mm'],['Origin distance', (Math.hypot(...xyz)*1000).toFixed(1)+' mm']]) {const a=document.createElement('span'),b=document.createElement('span');a.textContent=label;b.textContent=value;dl.append(a,b);}
  $('joint-detail').append(dl); updateDisplay();
}

function setMode(next) {
  mode=next;
  for(const id of ['live','pause','cad']) $(id).classList.toggle('active',mode===id);
  updatePose();
}
function takeReference(initial=false) {
  if (!latest) return;
  const readings=freshSnapshot(latest,performance.now()-receivedAt);
  if(initial && latest.initial_reference) reference=structuredClone(latest.initial_reference);
  else {
    if(disconnected || !latest.serial_open || !readings.every(r=>r.fresh)) return;
    reference={raw_deg:Object.fromEntries(readings.map(r=>[r.servo,r.angle_deg])),timestamp:latest.timestamp,sequence:latest.sequence};
  }
  $('reference-status').textContent='Rest captured '+new Date(reference.timestamp*1000).toLocaleTimeString()+'. Viewer reference only.';
  updatePose();
}

function updatePose() {
  if (!robot || !meta) return;
  const readings = latest ? freshSnapshot(latest,performance.now()-receivedAt) : [];
  const armFresh = !disconnected && latest?.serial_open && readings.length===7 && readings.slice(0,6).every(r=>r.fresh);
  if(mode==='cad') poseValues=Array(6).fill(0);
  else if(mode==='live' && reference && armFresh) {
    poseValues=meta.joints.map((j,i)=>relativeAngle(readings[i].angle_deg,reference.raw_deg[i],qReference[i],signs[i],meta.star_mapping[i],$('mapping').value));
  }
  meta.joints.forEach((j,i)=>robot.setJointValue(j.name,poseValues[i]));
  if(meta.gripper_joint_present) {
    const gripperFresh = !disconnected && latest?.serial_open && readings[6]?.fresh;
    if(mode==='cad')gapState={requested:0,applied:0};
    else if(mode==='live') {
      if(!$('follow-gripper').checked)gapState={requested:Number($('gap').value)/1000,applied:Number($('gap').value)/1000};
      else if(reference&&gripperFresh)gapState=gripperGap(readings[6].angle_deg,reference.raw_deg[6],meta.star_mapping[6],meta.revision.motor_calibration,$('flip-gripper').checked);
    }
    robot.setJointValue('gripper_left_joint',gapState.applied/2);
    // URDFLoader follows the right-hand mimic automatically.
    $('gap-value').textContent=(gapState.applied*1000).toFixed(2)+' mm';
    if($('follow-gripper').checked||mode!=='live')$('gap').value=gapState.applied*1000;
    $('gripper-status').textContent=mode==='cad'?'CAD zero: jaws closed.':mode==='pause'?'Gripper pose frozen.':!$('follow-gripper').checked?'Manual model control.':!gripperFresh?'Gripper held: waiting for fresh Star ID 6.':Math.abs(gapState.applied-gapState.requested)>.0001?'At model travel limit; requested '+(gapState.requested*1000).toFixed(1)+' mm.':'Following Star ID 6 · endpoint interpolation for visual inspection.';
  }
  if(mode==='cad') {$('pose-label').textContent='CAD zero · all joints 0°';$('pose-note').textContent='The designer’s exported pose. This is not necessarily the real arm’s rest pose.';}
  else if(mode==='pause') {$('pose-label').textContent='Frozen pose';$('pose-note').textContent='Sensors continue reading. The displayed model is held still.';}
  else if(!reference) {$('pose-label').textContent='Waiting for all seven Star readings';$('pose-note').textContent='The model remains at CAD zero. No readings are being invented.';}
  else if(!armFresh) {$('pose-label').textContent='Live pose held · readings missing';$('pose-note').textContent='Waiting for fresh readings from all six arm servos.';}
  else {$('pose-label').textContent='Following Star · relative to captured rest';$('pose-note').textContent=meta.gripper_joint_present?'Six arm joints + symmetric gripper. Check pivots, direction and resting shape.':'Rest angles and directions are unverified. Gripper motion is absent from this URDF.';}
  rowElements.forEach((el,i)=>{
    const r=readings[i],raw=r?.angle_deg;
    el.raw.textContent=Number.isFinite(raw)?raw.toFixed(1)+'°'+(r.fresh?'':' · stale'):'—';el.raw.className=r?.fresh?'':'stale';
    el.delta.textContent=reference&&Number.isFinite(raw)?(raw-reference.raw_deg[i]).toFixed(1)+'°':'—';
    if(i<6){const q=poseValues[i];el.q.textContent=deg(q).toFixed(1)+'°';el.q.className=withinLimits(q,meta.joints[i].limits)?'':'outside';el.q.title=withinLimits(q,meta.joints[i].limits)?'Within designer limits':'Outside designer limits — displayed without clamping';}
    else if(meta.gripper_joint_present){el.q.textContent=(gapState.applied*1000).toFixed(2)+' mm gap';el.q.className=Math.abs(gapState.applied-gapState.requested)>.0001?'outside':'';}
  });
  const fresh = readings.filter(r=>r.fresh).length;
  $('capture').disabled=!(fresh===7 && !disconnected && latest?.serial_open);
  const conn=$('connection');
  conn.textContent=disconnected?'Reader disconnected':fresh===7&&latest?.serial_open?'Star connected · 7/7 fresh':`Star · ${fresh}/7 fresh`;
  conn.className='badge '+(!disconnected&&fresh===7&&latest?.serial_open?'good':fresh>0?'warn':'bad');
  [...$('sample-meter').children].forEach((e,i)=>{e.classList.toggle('fresh',!!readings[i]?.fresh);e.title=readings[i]?.age_ms!==null&&readings[i]?.age_ms!==undefined?`${(readings[i].age_ms+performance.now()-receivedAt).toFixed(0)} ms since servo ${i} replied`:'No reply yet';});
  $('reader-message').textContent=disconnected?'Local reader is unavailable. Restart the viewer server.':fresh===7?'All seven sensors are responding.':latest?.error||'Some servo replies are missing. The model will hold if any arm reading becomes stale.';
  $('rate').textContent=(latest&&!disconnected?latest.hz.toFixed(0):'—')+' Hz';
}

async function poll() {
  if(pollInFlight)return;pollInFlight=true;
  try {
    const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(1200)});
    if(!response.ok)throw new Error('Reader unavailable');
    latest=await response.json();receivedAt=performance.now();disconnected=false;$('port').textContent=latest.port;
    if(!reference&&latest.initial_reference)takeReference(true);
  } catch(e){disconnected=true;}
  finally{pollInFlight=false;updatePose();}
}

function downloadReport() {
  const elapsed=performance.now()-receivedAt;
  const report={schema:1,created_at:new Date().toISOString(),purpose:'Visual and kinematic inspection; not a physics certification',
    source:{filename:meta.filename,sha256:meta.sha256,urdf_sha256:meta.urdf_sha256},
    reference_verified:false,reference,mode,angle_mapping:$('mapping').value,
    viewer_only:{up:$('up').value,base_lift_m:wrapper.position.z,reference_rad:qReference,directions:signs,
      hidden_visual_meshes:$('hide-loops').checked?[...decorativeLoopMeshes]:[],collision_and_mass_unchanged:true},
    current_readings:latest?freshSnapshot(latest,elapsed):null,reader_disconnected:disconnected,
    urdf_angles_rad:Object.fromEntries(meta.joints.map((j,i)=>[j.name,poseValues[i]])),
    gripper:meta.gripper_joint_present?{gap_m:gapState.applied,requested_gap_m:gapState.requested,jaw_coordinates_m:{gripper_left_joint:gapState.applied/2,gripper_right_joint:gapState.applied/2},follow_star:$('follow-gripper').checked,reverse:$('flip-gripper').checked,interpolation_verified:false}:null,
    sdk_targets_rad:latest?latest.readings.map((r,i)=>Number.isFinite(r.angle_deg)?sdkTarget(r.angle_deg,meta.star_mapping[i]):null):null,
    checks:Object.fromEntries([...document.querySelectorAll('[data-check]')].map(c=>[c.dataset.check,c.checked])),
    notes:$('notes').value,known_missing:[...(meta.gripper_joint_present?[]:['Gripper joint']),'Unassigned CAD part_003','Verified encoder-to-URDF zero mapping','Measured physical properties'],
    model:meta};
  const blob=new Blob([JSON.stringify(report,null,2)],{type:'application/json'});const url=URL.createObjectURL(blob);
  const a=document.createElement('a');a.href=url;a.download='maker-arm-urdf-inspection-'+new Date().toISOString().replace(/[:.]/g,'-')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}

async function init() {
  meta=await (await fetch('/api/model')).json();
  $('model-summary').textContent=`${meta.filename} · ${meta.links} links · ${(meta.all_joints||meta.joints).length} joints · ${meta.mass_kg.toFixed(2)} kg declared. ${meta.visual_meshes} visual / ${meta.collision_meshes} collision meshes.`;
  $('gripper-panel').hidden=!meta.gripper_joint_present;
  if(meta.gripper_joint_present){$('hide-loops').disabled=true;$('hide-loops').parentElement.title='Loop visuals are removed from the loaded URDF; collision shapes and mass remain.';}
  for(let i=0;i<7;i++){
    const row=document.createElement('tr'),name=document.createElement('td'),raw=document.createElement('td'),delta=document.createElement('td'),q=document.createElement('td'),limit=document.createElement('td');
    name.textContent=i===6?'Gripper':`${i+1}. ${meta.joint_labels[i]}`;const id=document.createElement('small');id.textContent='ID '+i;name.append(id);
    if(i<6){row.dataset.joint=String(i);row.onclick=()=>selectJoint(i);limit.textContent=deg(meta.joints[i].limits.lower).toFixed(0)+'° … '+deg(meta.joints[i].limits.upper).toFixed(0)+'°';const option=document.createElement('option');option.value=i;option.textContent=`${i+1}. ${meta.joint_labels[i]} · Star ID ${i}`;$('selected').append(option);}
    else if(meta.gripper_joint_present){row.onclick=()=>selectJoint(6);limit.textContent='0 … 104.825 mm gap';const option=document.createElement('option');option.value=6;option.textContent='7. Gripper · Star ID 6';$('selected').append(option);}
    else {q.textContent='Joint missing';q.className='outside';limit.textContent='—';}
    raw.textContent=delta.textContent='—';row.append(name,raw,delta,q,limit);$('joint-rows').append(row);rowElements.push({row,raw,delta,q});
    const indicator=document.createElement('span');indicator.textContent=i;$('sample-meter').append(indicator);
  }
  const manager=new THREE.LoadingManager();THREE.Cache.enabled=true;
  const meshReady=new Promise(resolve=>manager.onLoad=resolve);
  manager.onError=url=>errors.push(url);
  const loader=new URDFLoader(manager);loader.parseCollision=true;
  robot=await loader.loadAsync('/model/robot.urdf');await meshReady;
  if(errors.length)throw new Error('Failed to load meshes: '+errors.join(', '));
  wrapper.add(robot);
  const linkOrder=[meta.root,...(meta.all_joints||meta.joints).map(j=>j.child)];
  for(const [index,name] of linkOrder.entries()) {
    const link=robot.links[name];
    for(const group of link.children.filter(c=>c.isURDFVisual||c.isURDFCollider)) {
      const collision=!!group.isURDFCollider;
      group.traverse(mesh=>{if(!mesh.isMesh)return;
        mesh.material=new THREE.MeshStandardMaterial({color:collision?0xda8434:colors[index%colors.length],metalness:.28,roughness:.53,wireframe:collision,transparent:collision,opacity:collision?.55:1});
        const filename = group.urdfNode?.querySelector('geometry > mesh')?.getAttribute('filename');
        materialRecords.push({mesh,collision,linkIndex:name.startsWith('gripper_')?7:index,decorativeLoop:decorativeLoopMeshes.has(filename)});
      });
    }
  }
  meta.joints.forEach((j,i)=>{
    const joint=robot.joints[j.name];joint.ignoreLimits=true;
    const group=new THREE.Group();const axis=joint.axis.clone().normalize();
    const arrow=new THREE.ArrowHelper(axis,new THREE.Vector3(),.065,0xd38523,.013,.007);
    // Axis arrows remain attached to the actual URDF joint frames.
    group.add(arrow);const label=labelSprite(String(i+1));label.position.copy(axis).multiplyScalar(.08);group.add(label);joint.add(group);axisGroups.push(group);
  });
  const xml=new DOMParser().parseFromString(await (await fetch('/model/robot.urdf')).text(),'application/xml');
  for(const l of xml.querySelectorAll('robot > link')) {
    const origin=l.querySelector('inertial > origin'); if(!origin)continue;
    const point=new THREE.Mesh(new THREE.SphereGeometry(.005,12,8),new THREE.MeshBasicMaterial({color:0xd82972,depthTest:false}));point.position.set(...origin.getAttribute('xyz').trim().split(/\s+/).map(Number));point.renderOrder=90;robot.links[l.getAttribute('name')].add(point);comGroups.push(point);
  }
  resize();selectJoint(0);orientModel();updateDisplay();$('loading').hidden=true;
  $('selected').onchange=e=>selectJoint(Number(e.target.value));
  $('reference-angle').oninput=e=>{qReference[selected]=rad(Number(e.target.value));$('reference-value').textContent=Number(e.target.value).toFixed(1)+'°';updatePose();};
  $('flip').onchange=e=>{signs[selected]=e.target.checked?-1:1;updatePose();};
  $('reset-reference').onclick=()=>{qReference.fill(0);signs.fill(1);selectJoint(selected);updatePose();};
  $('mapping').onchange=updatePose;$('capture').onclick=()=>takeReference();
  for(const id of ['live','pause','cad'])$(id).onclick=()=>setMode(id);
  $('gap').oninput=()=>{$('follow-gripper').checked=false;setMode('live');};
  $('follow-gripper').onchange=updatePose;$('flip-gripper').onchange=updatePose;
  $('open-model-gripper').onclick=()=>{$('follow-gripper').checked=false;$('gap').value=104.825;setMode('live');selectJoint(6);};
  $('close-model-gripper').onclick=()=>{$('follow-gripper').checked=false;$('gap').value=0;setMode('live');selectJoint(6);};
  for(const id of ['axes','collision','com','hide-loops'])$(id).onchange=updateDisplay;
  $('up').onchange=orientModel;$('fit').onclick=()=>fitView();
  document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>fitView(b.dataset.view));$('report').onclick=downloadReport;
  await poll();setInterval(poll,80);
  // A local diagnostic hook for numerical checks; it never touches hardware.
  window.urdfBench={robot,meta,wrapper,scene,renderer,camera,controls,errors,get pose(){return [...poseValues]},get reference(){return reference},get latest(){return latest},get gripper(){return {...gapState}},setMode,selectJoint,fitView};
}
init().catch(e=>{$('loading').textContent='Viewer could not load: '+e.message;console.error(e);});
let lastUi=0;
function draw(t){requestAnimationFrame(draw);controls.update();renderer.render(scene,camera);if(t-lastUi>100){updatePose();lastUi=t;}}
requestAnimationFrame(draw);
