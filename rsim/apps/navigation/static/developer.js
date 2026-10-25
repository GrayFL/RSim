'use strict';
let settingsDocument=null;
// Settings contain JSON values only; Android's WebView 95 has no structuredClone.
function cloneSettings(value){return JSON.parse(JSON.stringify(value));}
const fieldLabels={max_linear:'线速度上限 · m/s',max_angular:'角速度上限 · rad/s',linear_acceleration:'驱动加速度上限 · m/s²',linear_deceleration:'刹车减速度上限 · m/s²',angular_acceleration:'角加速度上限 · rad/s²',angular_deceleration:'角减速度上限 · rad/s²',linear_friction:'线摩擦阻力 / 质量 · m/s²',linear_drag:'线速度平方阻力系数 · 1/m',angular_friction:'角摩擦阻力 · rad/s²',angular_drag:'角速度平方阻力系数',min_turn_radius:'牵引最小转弯半径 · m',rear_extent:'牵引后部距离 · m',rear_lateral_speed:'牵引后部横向速度上限 · m/s',max_loop_gap:'最大积分间隔 · s',wheelbase:'等效轴距 · m',steering_max_deg:'前轮转角范围 · °',steering_rate:'前轮转角速度上限 · rad/s',steering_acceleration:'前轮转角加速度 · rad/s²',steering_deceleration:'前轮转角减速度 · rad/s²',heading_max_deg:'相对朝向目标范围 · °'};
const modeNames={towing:'牵引',differential:'差速'},axisNames={velocity_target:'速度目标',acceleration:'加速度',angle_target:'角度目标'};
function controlConfiguration(values){controlsConfig={keyboard:{...values.keyboard},touch_mode:values.touch_mode,manual:cloneSettings(values.manual)};$('touchMode').value=values.touch_mode;$('touchButtons').hidden=values.touch_mode!=='buttons';$('joystick').hidden=values.touch_mode!=='joystick';updateControlLabels();}
function updateControlLabels(){const mode=latest.drive_mode||'towing',axes=controlsConfig.manual[mode];if(axes)$('axisHint').textContent=`Y · 线${axisNames[axes.y_mode]}　X · ${axisNames[axes.x_mode]}${axes.x_mode==='angle_target'?(axes.angle_reference==='steering'?'（前轮转角）':'（相对朝向）'):''}`;const k=controlsConfig.keyboard;$('keyHint').textContent=`${k.forward} / ${k.backward} 纵向，${k.left} / ${k.right} 转向，${k.stop} 停止。反向键先刹停，松开后再按才倒车。`;}
function field(parent,path,label,value,type='number',options=null){const item=document.createElement('label');item.textContent=label;const input=document.createElement(options?'select':'input');input.dataset.setting=path;if(options){for(const [v,text]of options){const option=document.createElement('option');option.value=v;option.textContent=text;input.append(option);}input.value=value;}else{input.type=type;if(type==='checkbox')input.checked=value;else input.value=value;if(type==='number'){input.step='any';input.min='0.001';}}item.append(input);parent.append(item);return input;}
function section(title,note){const wrap=document.createElement('section');wrap.className='settings-section';const h=document.createElement('h3');h.textContent=title;const p=document.createElement('p');p.className='hint';p.textContent=note;const grid=document.createElement('div');grid.className='settings-grid';wrap.append(h,p,grid);$('settingsFields').append(wrap);return grid;}
function renderSettings(){
  const d=settingsDocument,v=d.values;$('settingsFields').replaceChildren();
  let grid=section('功能许可','重启生效 · 开启运动许可后仍须取得控制权并手动使能；不会自动开始运动。');
  for(const [key,label]of Object.entries({motion:'允许发布非零运动',localization:'允许发布位置校准',mapping:'允许建图写入',pedal:'允许踏板推杆动作',audio:'允许录音和语音播报'}))field(grid,'permissions.'+key,label,v.permissions[key],'checkbox');
  grid=section('控制方式与按键','保存后生效 · 使用物理按键代码；点击按键框后按下新键。');
  field(grid,'touch_mode','触屏控制',v.touch_mode,'text',[['buttons','WASD 按键'],['joystick','方形摇杆']]);
  field(grid,'drive_mode','默认运动模式',v.drive_mode,'text',Object.entries(modeNames));
  for(const [key,label]of Object.entries({forward:'前进 / Y+',backward:'刹车或倒车 / Y−',left:'向左 / X+',right:'向右 / X−',stop:'停止 / 解除使能'})){
    const input=field(grid,'keyboard.'+key,label,v.keyboard[key],'text');input.readOnly=true;input.onkeydown=e=>{e.preventDefault();input.value=e.code;};
  }
  for(const mode of ['towing','differential']){
    grid=section(modeNames[mode]+'轴绑定','保存后生效 · 绑定与运动模型独立；相对朝向以本次按住遥控时的车头方向为零。');
    field(grid,`manual.${mode}.y_mode`,'Y 轴',v.manual[mode].y_mode,'text',[['acceleration','线加速度'],['velocity_target','线速度目标']]);
    field(grid,`manual.${mode}.x_mode`,'X 轴',v.manual[mode].x_mode,'text',[['angle_target','角度目标'],['velocity_target','角速度目标'],['acceleration','角加速度']]);
    field(grid,`manual.${mode}.angle_reference`,'角度目标含义',v.manual[mode].angle_reference,'text',[['steering','前轮转角'],['heading','相对车体朝向']]);
    grid=section(modeNames[mode]+'物理参数','重启生效 · SI 单位；最大硬件范围为 1.6 m/s、0.8 rad/s。驱动加速度需大于摩擦阻力。');
    for(const [key,label]of Object.entries(fieldLabels))field(grid,`profiles.${mode}.${key}`,label,v.profiles[mode][key]);
  }
  grid=section('速度与碰撞外形','保存后生效 · 速度仍受所选模型和实时硬件上限约束；外形不触发推杆动作。');
  for(const [key,label]of Object.entries({navigation:'导航速度上限 · m/s',manual:'手动速度上限 · m/s',angular:'角速度上限 · rad/s'}))field(grid,'speed_limits.'+key,label,v.speed_limits[key]);
  field(grid,'speed_limits.handles','允许把手调速',v.speed_limits.handles,'checkbox');
  for(const [key,label]of Object.entries({front:'前部距离 · m',rear:'后部距离 · m',width:'宽度 · m',margin:'碰撞余量 · m'}))field(grid,'footprint.'+key,label,v.footprint[key]);
  $('settingsStatus').textContent=d.restart_required.length?'已保存的物理参数或功能许可等待重启。':'已保存设置与当前服务一致。';
  $('restartService').disabled=!d.restart_available;
}
async function loadSettings(render=false){settingsDocument=await api('/api/settings');controlConfiguration(settingsDocument.active);if(render)renderSettings();return settingsDocument;}
async function commitSettings(values){if(!controlReady)throw Error('请先取得控制权');clearManual();settingPending=true;controlButtons();try{const d=await api('/api/settings',auxEnvelope({revision:settingsDocument.revision,values}));settingsDocument={...d,restart_available:settingsDocument.restart_available};controlConfiguration(d.active);return d;}finally{settingPending=false;controlButtons();}}
$('openDeveloper').onclick=async()=>{clearManual();stop();try{await loadSettings(true);$('developer').showModal();}catch(e){notice(e.message);}};
$('closeDeveloper').onclick=()=>$('developer').close();
$('reloadSettings').onclick=()=>loadSettings(true).catch(e=>{$('settingsStatus').textContent=e.message;});
$('saveSettings').onclick=async()=>{try{const values=cloneSettings(settingsDocument.values);for(const input of $('settingsFields').querySelectorAll('[data-setting]')){const path=input.dataset.setting.split('.');let object=values;for(const key of path.slice(0,-1))object=object[key];object[path[path.length-1]]=input.type==='checkbox'?input.checked:input.type==='number'?Number(input.value):input.value;}await commitSettings(values);renderSettings();$('settingsStatus').textContent=settingsDocument.restart_required.length?'设置已保存；物理参数或功能许可等待重启。':'设置已保存并应用，运动已解除使能。';}catch(e){$('settingsStatus').textContent=e.message;}};
$('touchMode').onchange=async()=>{try{const selected=$('touchMode').value;await loadSettings();const values=cloneSettings(settingsDocument.values);values.touch_mode=selected;await commitSettings(values);}catch(e){notice(e.message);controlConfiguration(controlsConfig);}};
$('restartService').onclick=async()=>{try{clearManual();settingPending=true;controlButtons();$('restartService').disabled=true;await api('/api/service/restart',auxEnvelope());$('settingsStatus').textContent='正在重启 RSim；连接恢复后请重新取得控制权。';if(socket)socket.close();await new Promise(r=>setTimeout(r,2000));for(let i=0;i<40;i++){await new Promise(r=>setTimeout(r,1000));try{const d=await api('/api/settings');if(d.restart_available&&!d.restart_required.length){settingsDocument=d;controlConfiguration(d.active);renderSettings();await poll();notice('RSim 已重新启动，运动未使能。');return;}}catch(_){}}throw Error('重启等待超时，请检查启动日志。');}catch(e){$('settingsStatus').textContent=e.message;}finally{settingPending=false;controlButtons();}};
loadSettings().catch(()=>{});
