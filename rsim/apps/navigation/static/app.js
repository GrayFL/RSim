'use strict';
const $ = id => document.getElementById(id);
let socket=null, lease=null, seq=0, offset=0, held=null, keys=new Set(), latest={}, meta=null, mapImage=null;
let visual={}, view={x:0,y:0,scale:30}, staged=null, catalog={maps:[],current_map_uuid:null}, waypoints=[];
let lastPing=0,lastStatus=0,lastVisual=0,pollBusy=false,visualBusy=false,imageKey='',mapRequest=0;
let controlReady=false,settingPending=false,controlLag=false;
let clockSamples=[];
function observeClock(clock){const now=performance.now();clockSamples=clockSamples.filter(p=>now-p.at<10000);clockSamples.push({at:now,offset:clock-now});if(clockSamples.length>80)clockSamples.shift();offset=Math.max(...clockSamples.map(p=>p.offset));}
function controlButtons(){for(const id of ['goal','driveMode','extended','resume','saveSpeeds'])$(id).disabled=!controlReady||settingPending;$('arm').disabled=latest.zero_only!==false||!controlReady||settingPending;}
function notice(text){$('notification').textContent=text;}
function controlNotice(result){
  if(result.error){
    controlLag=['heartbeat','manual'].includes(result.op)&&/deadline|expired/.test(result.error);
    notice(controlLag?'控制消息延迟，等待有效输入。':result.error);return;
  }
  if(!result.ok||!result.op)return;
  if(['heartbeat','manual'].includes(result.op)){if(controlLag){controlLag=false;notice('控制通道已恢复。');}return;}
  controlLag=false;
  notice(({arm:'运动已使能',goal:'导航目标已接收',drive_mode:'运动模式已切换',speeds:'速度设置已保存',footprint:'碰撞外形已更新',stop:'已停止并解除使能',pause:'任务已暂停',resume:'任务已恢复',cancel:'任务已取消'})[result.op]||'操作已完成');
}
async function api(path,body){
  const response=await fetch(path,{method:body===undefined?'GET':'POST',headers:body===undefined?{}:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),cache:'no-store'});
  if(response.status===401){if(!$('login').open)$('login').showModal();throw Error('请输入访问令牌');}
  const result=await response.json();
  if(!response.ok||result.ok===false)throw Error(typeof result.detail==='string'?result.detail:(result.error||'请求失败'));
  return result;
}
function envelope(op,extra={}){return {op,lease,sequence:++seq,deadline_ms:performance.now()+offset+(['goal','footprint','drive_mode','speeds'].includes(op)?1600:400),...extra};}
function clearManual(){held=null;keys.clear();document.querySelectorAll('.held').forEach(e=>e.classList.remove('held'));}
function send(op,extra={}){
  if(['stop','pause','cancel'].includes(op))clearManual();
  if(controlReady&&socket&&socket.readyState===WebSocket.OPEN&&socket.bufferedAmount<4096){if(['arm','drive_mode','footprint','speeds'].includes(op)){settingPending=true;controlButtons();}socket.send(JSON.stringify(envelope(op,extra)));return;}
  if(['stop','pause','cancel'].includes(op))api('/api/control',{op}).catch(e=>notice(e.message));else notice('请先取得控制权');
}
function stop(){send('stop');}
$('loginForm').addEventListener('submit',async e=>{e.preventDefault();try{await api('/api/session',{token:$('token').value});$('token').value='';$('login').close();await poll();await loadMaps();}catch(error){$('loginError').textContent=error.message;}});
$('claim').onclick=async()=>{try{
  const result=await api('/api/claim',{});lease=result.lease;seq=0;clockSamples=[];observeClock(result.clock_ms);
  if(socket)socket.close();const channel=new WebSocket((location.protocol==='https:'?'wss:':'ws:')+'//'+location.host+'/ws/control');socket=channel;
  channel.onmessage=e=>{const result=JSON.parse(e.data);if(result.clock_ms)observeClock(result.clock_ms);if(['arm','drive_mode','footprint','speeds'].includes(result.op)){settingPending=false;controlButtons();}controlNotice(result);if(result.hello){controlReady=true;controlLag=false;controlButtons();send('heartbeat');$('leaseState').textContent='当前浏览器持有';notice('控制已连接。');}};
  channel.onclose=()=>{if(socket!==channel)return;lease=null;controlReady=false;settingPending=false;controlButtons();clearManual();$('leaseState').textContent='连接已断开';notice('控制连接断开，机器人停止。');};
}catch(e){notice(e.message);}};
$('arm').onclick=()=>send('arm');$('stop').onclick=stop;
for(const op of ['pause','resume','cancel'])$(op).onclick=()=>send(op);
function selectedMap(){return $('mapSelect').value||catalog.current_map_uuid||null;}
function showingLive(){return !$('mapSelect').value||$('mapSelect').value===catalog.current_map_uuid;}
function goalValues(){const out={x:Number($('gx').value),y:Number($('gy').value),yaw:Number($('gyaw').value)};if(selectedMap())out.map_uuid=selectedMap();if(latest.map)out.map_version=latest.map.version;return out;}
function setGoal(p){staged=[p.x,p.y,p.yaw];for(const [id,k]of [['gx','x'],['gy','y'],['gyaw','yaw']])$(id).value=Number(p[k]).toFixed(3);draw();}
$('goal').onclick=()=>{if(!showingLive())return notice('请切回机器人当前地图后导航。');send('goal',goalValues());};
$('localize').onclick=async()=>{if($('mapMode').value!=='localize')return notice('请先进入“校正机器人位置”模式，在地图上选取位置和朝向。');if(!showingLive())return notice('只能在当前地图校正位置。');try{await api('/api/localization',envelope('goal',goalValues()));notice('位置校正已提交');}catch(e){notice(e.message);}};
$('extended').onchange=()=>send('footprint',{extended:$('extended').checked});
$('driveMode').onchange=()=>{clearManual();send('drive_mode',{drive_mode:$('driveMode').value});};
$('scale').oninput=()=>{$('scaleText').textContent=$('scale').value+'%';};
function axes(){
  const scale=Number($('scale').value)/100;
  if(held)return {throttle:held.throttle*scale,steering:held.steering*scale};
  const differential=latest.drive_mode==='differential';
  return {throttle:(Number(keys.has('w'))-Number(differential&&keys.has('s')))*scale,steering:(Number(keys.has('a'))-Number(keys.has('d')))*scale};
}
function manualHeld(){return held!==null||keys.has('w')||(latest.drive_mode==='differential'&&['a','s','d'].some(k=>keys.has(k)));}
document.querySelectorAll('.drive').forEach(button=>{
  button.addEventListener('pointerdown',e=>{e.preventDefault();button.setPointerCapture(e.pointerId);held={throttle:Number(button.dataset.throttle),steering:Number(button.dataset.steer)};button.classList.add('held');send('manual',axes());});
  for(const event of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(event,()=>{if(held!==null){clearManual();send('manual',{throttle:0,steering:0});}});
});
document.addEventListener('keydown',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(document.activeElement.tagName))return;const key=e.key.toLowerCase();if(key===' '){e.preventDefault();stop();}if(['w','a','s','d'].includes(key)){e.preventDefault();if(key==='s'&&latest.drive_mode!=='differential'){stop();return;}keys.add(key);}});
document.addEventListener('keyup',e=>{const key=e.key.toLowerCase();if(!keys.delete(key))return;if(!manualHeld())send('manual',{throttle:0,steering:0});});
document.addEventListener('visibilitychange',()=>{if(document.hidden){stop();if(socket)socket.close();}});
window.addEventListener('blur',()=>{clearManual();stop();});
window.addEventListener('pagehide',()=>{navigator.sendBeacon('/api/control',new Blob([JSON.stringify({op:'stop'})],{type:'application/json'}));if(socket)socket.close();});
setInterval(()=>{if(!lease||document.hidden||!socket||socket.readyState!==WebSocket.OPEN)return;const now=performance.now();if(manualHeld()){send('manual',axes());lastPing=now;}else if(now-lastPing>200){send('heartbeat');lastPing=now;}},80);
async function poll(){if(pollBusy||document.hidden)return;pollBusy=true;try{
  latest=await api('/api/status');$('dot').className=latest.hardware.health==='bridge_unavailable'?'':'live';
  $('connection').textContent=lease?'控制通道在线':'状态已连接';$('modeBadge').textContent=latest.simulated?'仿真环境':latest.zero_only?'零速保护':latest.armed?'运动已使能':'运动未使能';
  $('reason').textContent=latest.mode+' · '+latest.reason;$('speed').textContent=latest.measured?Number(latest.measured[0]).toFixed(3):'—';$('yawSpeed').textContent=latest.measured?Number(latest.measured[1]).toFixed(3):'—';$('latency').textContent=latest.loop_ms;
  $('estop').textContent=latest.hardware.estop===true?'已按下':latest.hardware.estop===false?'未按下':'未知';$('gate').textContent=latest.hardware.health;controlButtons();
  $('armHint').textContent=latest.zero_only?'零速部署：可预览规划，实际速度保持为零。':'松开遥控或断开控制连接会停车。';
  if(latest.footprint)$('extended').checked=latest.footprint.rear>.3;
  $('driveMode').value=latest.drive_mode||'towing';document.querySelector('.differential').hidden=latest.drive_mode!=='differential';
  updateAuxiliary();
  const chosen=$('mapSelect').value?catalog.maps.find(m=>m.map_uuid===$('mapSelect').value):latest.map;
  if(chosen&&(!meta||chosen.version!==meta.version)){meta=chosen;mapImage=null;imageKey='';if(latest.pose&&showingLive()){view.x=latest.pose[0];view.y=latest.pose[1];}}
  draw();
}catch(e){notice(e.message);}finally{pollBusy=false;}}
$('low').onchange=()=>notice($('low').checked?'低带宽刷新，驾驶响应频率保持不变。':'标准刷新频率');
$('visuals').onchange=()=>{document.body.classList.toggle('hidden-visuals',!$('visuals').checked);$('mapEmpty').hidden=$('visuals').checked;if($('visuals').checked)refreshVisual();else{visual={};draw();}};
async function refreshVisual(){if(visualBusy||!$('visuals').checked||document.hidden)return;visualBusy=true;try{
  const key=($('mapSelect').value||'live')+':'+(meta?.version||'');
  if(meta&&(!mapImage||imageKey!==key)){
    const path=$('mapSelect').value?'/api/maps/'+encodeURIComponent($('mapSelect').value)+'/image':'/api/map.png';
    const response=await fetch(path);if(!response.ok)throw Error('地图尚未就绪');
    const image=new Image(),url=URL.createObjectURL(await response.blob());
    try{await new Promise((resolve,reject)=>{image.onload=resolve;image.onerror=reject;image.src=url;});if(key===($('mapSelect').value||'live')+':'+(meta?.version||'')){mapImage=image;imageKey=key;}}finally{URL.revokeObjectURL(url);}
  }
  visual=await api('/api/visual');draw();
}catch(e){notice(e.message);}finally{visualBusy=false;}}
setInterval(()=>{const now=performance.now();if(now-lastStatus>($('low').checked?2000:500)){lastStatus=now;poll();}if(now-lastVisual>($('low').checked?2500:500)){lastVisual=now;refreshVisual();}},200);
const canvas=$('map'),ctx=canvas.getContext('2d');
function toScreen(p){return [(p[0]-view.x)*view.scale+canvas.clientWidth/2,canvas.clientHeight/2-(p[1]-view.y)*view.scale];}
function toWorld(p){return [(p[0]-canvas.clientWidth/2)/view.scale+view.x,view.y-(p[1]-canvas.clientHeight/2)/view.scale];}
function line(points,color,width=2,closed=false){if(!points||!points.length)return;ctx.beginPath();points.forEach((p,i)=>{const q=toScreen(p);i?ctx.lineTo(...q):ctx.moveTo(...q);});if(closed)ctx.closePath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
function arrow(p,color){const q=toScreen(p);ctx.save();ctx.translate(...q);ctx.rotate(-p[2]);ctx.fillStyle=color;ctx.beginPath();ctx.moveTo(12,0);ctx.lineTo(-7,-7);ctx.lineTo(-3,0);ctx.lineTo(-7,7);ctx.closePath();ctx.fill();ctx.restore();}
function draw(){
  const dpr=window.devicePixelRatio||1;canvas.width=canvas.clientWidth*dpr;canvas.height=canvas.clientHeight*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,canvas.clientWidth,canvas.clientHeight);if(!$('visuals').checked)return;
  if(mapImage&&meta){const o=toScreen(meta.origin);ctx.save();ctx.translate(...o);ctx.rotate(-meta.yaw);ctx.imageSmoothingEnabled=false;const size=meta.resolution*view.scale;ctx.drawImage(mapImage,0,-meta.height*size,meta.width*size,meta.height*size);ctx.restore();}
  for(const point of waypoints){const p=toScreen([point.x,point.y]);ctx.fillStyle='#4b779a';ctx.beginPath();ctx.arc(...p,4,0,Math.PI*2);ctx.fill();ctx.font='11px sans-serif';ctx.fillStyle='#224963';ctx.fillText(point.name,p[0]+7,p[1]-6);}
  if(showingLive()){line(visual.path,'#2387d2',3);line(visual.trajectory,'#13b88c',2);if($('scans').checked){ctx.fillStyle='#d55e49';for(const p of visual.scan||[]){const q=toScreen(p);ctx.fillRect(q[0],q[1],2,2);}}line(visual.footprint,'#168f6a',2,true);if(latest.pose)arrow(latest.pose,'#168b65');}
  if(staged)arrow(staged,'#df9414');
}
new ResizeObserver(draw).observe(canvas);
function zoom(factor,anchor=[canvas.clientWidth/2,canvas.clientHeight/2]){const world=toWorld(anchor);view.scale=Math.max(3,Math.min(500,view.scale*factor));view.x=world[0]-(anchor[0]-canvas.clientWidth/2)/view.scale;view.y=world[1]+(anchor[1]-canvas.clientHeight/2)/view.scale;draw();}
$('zoomIn').onclick=()=>zoom(1.4);$('zoomOut').onclick=()=>zoom(1/1.4);
$('center').onclick=()=>{if(latest.pose&&showingLive()){view.x=latest.pose[0];view.y=latest.pose[1];draw();}};
function fit(){if(!meta)return;const w=meta.width*meta.resolution,h=meta.height*meta.resolution,c=Math.cos(meta.yaw),s=Math.sin(meta.yaw);view.x=meta.origin[0]+c*w/2-s*h/2;view.y=meta.origin[1]+s*w/2+c*h/2;view.scale=.9*Math.min(canvas.clientWidth/(Math.abs(c)*w+Math.abs(s)*h),canvas.clientHeight/(Math.abs(s)*w+Math.abs(c)*h));draw();}
$('fit').onclick=fit;
function pointer(e){const b=canvas.getBoundingClientRect();return [e.clientX-b.left,e.clientY-b.top];}
const pointers=new Map();let gesture=null;
function pair(){const [a,b]=[...pointers.values()];return {center:[(a[0]+b[0])/2,(a[1]+b[1])/2],distance:Math.max(1,Math.hypot(a[0]-b[0],a[1]-b[1]))};}
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom(Math.exp(-e.deltaY*.001),pointer(e));},{passive:false});
canvas.onpointerdown=e=>{
  if(!$('visuals').checked)return;e.preventDefault();canvas.setPointerCapture(e.pointerId);const p=pointer(e);pointers.set(e.pointerId,p);
  if(pointers.size>=2){const p2=pair();gesture={kind:'pinch',...p2,world:toWorld(p2.center),scale:view.scale};return;}
  if($('mapMode').value==='browse')gesture={kind:'pan',start:p,view:{...view}};
  else{gesture={kind:'select',start:toWorld(p)};staged=[...gesture.start,Number($('gyaw').value)];draw();}
};
canvas.onpointermove=e=>{
  if(!pointers.has(e.pointerId)||!gesture)return;pointers.set(e.pointerId,pointer(e));
  if(gesture.kind==='pinch'&&pointers.size>=2){const p=pair();view.scale=Math.max(3,Math.min(500,gesture.scale*p.distance/gesture.distance));view.x=gesture.world[0]-(p.center[0]-canvas.clientWidth/2)/view.scale;view.y=gesture.world[1]+(p.center[1]-canvas.clientHeight/2)/view.scale;}
  else if(gesture.kind==='pan'){const p=pointer(e);view.x=gesture.view.x-(p[0]-gesture.start[0])/view.scale;view.y=gesture.view.y+(p[1]-gesture.start[1])/view.scale;}
  else if(gesture.kind==='select'){const p=toWorld(pointer(e)),d=Math.hypot(p[0]-gesture.start[0],p[1]-gesture.start[1]);if(d>.03)staged=[...gesture.start,Math.atan2(p[1]-gesture.start[1],p[0]-gesture.start[0])];}
  draw();
};
function finishPointer(e,cancelled=false){
  if(!pointers.has(e.pointerId))return;pointers.delete(e.pointerId);
  if(!cancelled&&gesture?.kind==='select'&&staged){setGoal({x:staged[0],y:staged[1],yaw:staged[2]});notice($('mapMode').value==='localize'?'位置与朝向已选择，点击“确认位置校正”。':$('mapMode').value==='mark'?'输入点位名称后点击“保存选点”。':'目标已选择，点击“前往此处”。');}
  if(pointers.size===1)gesture={kind:'pan',start:[...pointers.values()][0],view:{...view}};else gesture=null;
}
canvas.onpointerup=e=>finishPointer(e);canvas.onpointercancel=e=>finishPointer(e,true);canvas.onlostpointercapture=e=>finishPointer(e,true);
$('mapMode').onchange=()=>{gesture=null;pointers.clear();$('mapHint').textContent={browse:'拖动平移；滚轮或双指缩放。',goal:'按下选择目标，拖动设置朝向；确认后导航。',localize:'选取机器人真实位置和朝向；确认后更新定位。',mark:'选择新点位和朝向，输入名称后保存。'}[$('mapMode').value];};
async function loadMaps(){try{catalog=await api('/api/maps');const chosen=$('mapSelect').value;$('mapSelect').replaceChildren(new Option('实时地图',''));for(const map of catalog.maps)$('mapSelect').add(new Option(map.name+(map.map_uuid===catalog.current_map_uuid?' · 当前':''),map.map_uuid));if(catalog.maps.some(m=>m.map_uuid===chosen))$('mapSelect').value=chosen;await loadWaypoints();}catch(e){notice(e.message);}}
$('refreshMaps').onclick=loadMaps;
$('mapSelect').onchange=async()=>{mapRequest++;mapImage=null;imageKey='';staged=null;meta=$('mapSelect').value?catalog.maps.find(m=>m.map_uuid===$('mapSelect').value):latest.map;fit();await loadWaypoints();await refreshVisual();notice(showingLive()?'当前地图':'浏览历史地图；导航和位置校正仅适用于当前地图。');};
async function loadWaypoints(){const ticket=++mapRequest;try{const id=selectedMap();const points=await api('/api/waypoints'+(id?'?map_uuid='+encodeURIComponent(id):''));if(ticket!==mapRequest)return;waypoints=points;$('waypoints').replaceChildren(new Option('选择位置',''));waypoints.forEach((p,i)=>$('waypoints').add(new Option(p.name+(p.source==='oem'?' · 原有':''),String(i))));draw();}catch(e){notice(e.message);}}
$('waypoints').onchange=()=>{if($('waypoints').value==='')return;const p=waypoints[Number($('waypoints').value)];if(!p)return;setGoal(p);$('waypointName').value=p.name;$('renameWaypoint').disabled=$('deleteWaypoint').disabled=!p.editable;};
$('saveWaypoint').onclick=async()=>{try{if(!showingLive())throw Error('请在当前地图标点');await api('/api/waypoints',{...goalValues(),name:$('waypointName').value||'常用位置'});await loadWaypoints();notice('新点位已保存');}catch(e){notice(e.message);}};
for(const [id,remove]of [['renameWaypoint',false],['deleteWaypoint',true]])$(id).onclick=async()=>{try{if($('waypoints').value==='')throw Error('请先选择自建点位');const point=waypoints[Number($('waypoints').value)];if(!point.editable)throw Error('原有点位保持只读');await api('/api/waypoints/'+encodeURIComponent(point.id),remove?{delete:true}:{name:$('waypointName').value});await loadWaypoints();notice(remove?'点位已删除':'点位已改名');}catch(e){notice(e.message);}};
for(const [id,op]of [['mappingStart','start'],['mappingSave','save'],['mappingCancel','cancel']])$(id).onclick=async()=>{try{const result=await api('/api/mapping/'+op,{lease,sequence:++seq,deadline_ms:performance.now()+offset+1200,map_name:$('mapName').value||'RSim map'});$('mappingStatus').textContent=JSON.stringify(result);if(op==='save')loadMaps();}catch(e){notice(e.message);}};
$('mapEmpty').hidden=true;poll().then(loadMaps);
controlButtons();

function auxEnvelope(extra={}){return {lease,sequence:++seq,deadline_ms:performance.now()+offset+1600,...extra};}
function updateAuxiliary(){
  const t=latest.telemetry||{},b=t.battery||{},h=t.handles||{},handles=h.fresh?h.data?.data:null;
  $('battery').textContent=b.fresh&&b.data?.soc_percent!=null?Number(b.data.soc_percent).toFixed(0)+'%':'未知';
  $('charging').textContent=b.fresh?(b.data?.is_charging||b.data?.charging_state?'充电中':'未充电'):'未知';
  $('leftHandle').textContent=handles?.length>=2?(handles[0]?'按下':'松开'):'未知';$('rightHandle').textContent=handles?.length>=2?(handles[1]?'按下':'松开'):'未知';
  $('telemetryAge').textContent='电池数据 '+(b.age_sec==null?'未接收':b.age_sec+' 秒前')+' · 把手数据 '+(h.age_sec==null?'未接收':h.age_sec+' 秒前');
  const c=latest.capabilities||{};
  for(const id of ['pedalExtend','pedalRetract','pedalStop'])$(id).disabled=!c.pedal;
  for(const id of ['mappingStart','mappingSave','mappingCancel'])$(id).disabled=!c.mapping;
  for(const id of ['voiceOnce','voiceStart','voiceAnnounce','voiceVolume'])$(id).disabled=!c.audio;
  $('localize').disabled=!c.localization;
  if(!c.mapping)$('mappingStatus').textContent='当前部署关闭建图写入；实时地图仍可浏览。';
  if(!c.pedal)$('pedalStatus').textContent='当前部署关闭推杆动作；碰撞外形可独立设置。';
  const limits=latest.speed_limits||{};
  for(const [id,key]of [['navLimit','navigation'],['manualLimit','manual'],['angularLimit','angular']]){
    if(!$(id).dataset.edited&&document.activeElement!==$(id)&&limits[key]!=null)$(id).value=limits[key];
  }
}
for(const id of ['navLimit','manualLimit','angularLimit'])$(id).oninput=()=>{$(id).dataset.edited='1';};
$('saveSpeeds').onclick=()=>{send('speeds',{navigation:Number($('navLimit').value),manual:Number($('manualLimit').value),angular:Number($('angularLimit').value),handles:$('handleControl').checked});for(const id of ['navLimit','manualLimit','angularLimit'])delete $(id).dataset.edited;};
for(const [id,action]of [['pedalExtend','extend'],['pedalRetract','retract'],['pedalStop','stop']])$(id).onclick=async()=>{try{await api('/api/pushrod',auxEnvelope({action}));$('pedalStatus').textContent='动作已发送，位置尚未确认；继续按伸出外形避障。';}catch(e){notice(e.message);}};
for(const [id,op]of [['voiceSend','text'],['voiceOnce','once'],['voiceStart','start'],['voiceStop','stop']])$(id).onclick=async()=>{try{const result=await api('/api/assistant/'+op,auxEnvelope({text:$('voiceText').value}));$('voiceStatus').textContent=result.skipped||'请求已处理，识别到的指令将交给 RSim。';await refreshVoice();}catch(e){notice(e.message);}};
$('voiceVolume').oninput=()=>{$('volumeText').textContent=$('voiceVolume').value+'%';};
$('voiceVolume').onchange=async()=>{try{await api('/api/voice/volume',auxEnvelope({volume_pct:Number($('voiceVolume').value)}));notice('音量已更新');}catch(e){notice(e.message);}};
$('voiceAnnounce').onclick=async()=>{try{await api('/api/voice/announce',auxEnvelope({key:'depart_generic'}));notice('播报已完成');}catch(e){notice(e.message);}};
let voiceBusy=false;
async function refreshVoice(){if(voiceBusy||document.hidden)return;voiceBusy=true;try{const v=await api('/api/assistant/status');$('voicePhase').textContent=v.running?'正在监听':'未启动';if(v.last_dispatch)$('voiceStatus').textContent=v.last_dispatch.ok?'指令已交给 RSim：'+v.last_dispatch.action:v.last_dispatch.error;else if(v.available===false)$('voiceStatus').textContent='当前环境未配置语音模块。';if(v.volume&&document.activeElement!==$('voiceVolume')){$('voiceVolume').value=v.volume.volume_pct;$('volumeText').textContent=v.volume.volume_pct+'%';}}catch(e){notice(e.message);}finally{voiceBusy=false;}}
setInterval(refreshVoice,4000);
