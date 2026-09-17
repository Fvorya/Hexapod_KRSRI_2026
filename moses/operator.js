'use strict';
const opOwner = Math.random().toString(36).slice(2) + Date.now().toString(36);
const opFields = [
  ['Tinggi langkah', 'mm', 0, 120, 1], ['Panjang langkah', 'mm', 0, 150, 1],
  ['Waktu satu siklus', 'ms', 300, 3000, 25], ['Tinggi badan', 'mm', 40, 160, 1],
  ['Radius kaki', 'mm', 30, 120, 1]
];
let opData = null, opSeq = 0, opVector = null, opBusy = false, opSeen = 0;
let opDirty = false, opApplying = null, opRequested = null, opProfileKey = '';
let opLocalMessageUntil = 0, opReadRequested = false;
const opKeys = new Set();
const opLabels = {
  'gait.step_height':'Tinggi langkah dasar', 'gait.step_length':'Panjang langkah dasar',
  'gait.cycle_time':'Waktu siklus dasar', 'gait.duty':'Porsi kaki menapak',
  'gait.slew_rate':'Laju perubahan kecepatan', 'gait.profile_tau':'Waktu transisi profil',
  'gait.settle_tau':'Waktu kembali diam', 'wall.kp':'Koreksi jarak dinding (P)',
  'wall.kd':'Redaman koreksi dinding (D)', 'wall.setpoint':'Jarak target dinding (cm)',
  'wall.min':'Jarak minimum dinding (cm)', 'heading.kp':'Koreksi arah (P)',
  'heading.kd':'Redaman arah (D)', 'condong.mm':'Condong maju (mm)',
  'condong.jeda':'Jeda condong (ms)', 'condong.yaw':'Luruskan arah saat condong',
  'pulse.min':'Pulsa minimum servo kaki', 'pulse.max':'Pulsa maksimum servo kaki',
  'arm.pulse_min':'Pulsa minimum servo lengan', 'arm.pulse_max':'Pulsa maksimum servo lengan',
  'stab.tau':'Waktu respons stabilisasi', 'stab.sign_roll':'Arah koreksi roll',
  'stab.sign_pitch':'Arah koreksi pitch', 'arena.mirror':'Arena dicerminkan'
};
function opHuman(name) {
  if(opLabels[name]) return opLabels[name];
  const words = name.replace(/[._]/g,' ').replace(/\bpx\b/g,'piksel').replace(/\bdeg\b/g,'derajat')
    .replace(/\bconf\b/g,'keyakinan').replace(/\btol\b/g,'toleransi').replace(/\bmin\b/g,'minimum')
    .replace(/\bmax\b/g,'maksimum').replace(/\bfrac\b/g,'fraksi').replace(/\bms\b/g,'milidetik');
  return words.charAt(0).toUpperCase()+words.slice(1);
}
function opNotice(message) {
  $('op-notice').textContent = message;
  opLocalMessageUntil = performance.now()+3500;
}
async function opSend(k, data={}) {
  const controller = new AbortController(), timer = setTimeout(()=>controller.abort(),1800);
  try {
    const r=await fetch('/cmd?k='+encodeURIComponent(k)+'&v='+encodeURIComponent(JSON.stringify(data)),
      {method:'POST',signal:controller.signal});
    if(!r.ok) throw Error('HTTP '+r.status);
    return true;
  } catch(e) {
    opVector=null; opKeys.clear();
    opNotice('Perintah tidak terkonfirmasi. Periksa koneksi robot.');
    return false;
  } finally {clearTimeout(timer);}
}
function opReadSoon(){setTimeout(()=>opSend('op_read'),1800);}
function opHasControl(){return opData && opData.port_ok && opData.manual && opData.manual_owner===opOwner && !opData.halt && opData.umur>=0 && opData.umur<1.2 && performance.now()-opSeen<1200;}
function opCanDrive(){return opHasControl()&&opData.operator?.servo===true;}
async function opEnter(){
  if(opHasControl()) {opRelease(); await opSend('op_exit',{owner:opOwner,seq:++opSeq});}
  else await opSend('op_enter',{owner:opOwner});
}
async function opTick(){
  if(!opVector || opBusy) return;
  if(!opCanDrive()){opRelease();return;}
  opBusy=true;
  const speed=Number($('op-speed').value)/100;
  await opSend('op_jog',{owner:opOwner,seq:++opSeq,vector:opVector.map(n=>n*speed)});
  opBusy=false;
}
function opDrive(vector){
  if(!opCanDrive()){opNotice('Baca robot, lepas STOP jika terkunci, lalu aktifkan manual.');return;}
  opVector=vector;
  opTick();
}
function opRelease(){
  const wasMoving=Boolean(opVector);
  opVector=null;opKeys.clear();
  document.querySelectorAll('[data-drive]').forEach(b=>b.classList.remove('held'));
  if(wasMoving || opHasControl()) opSend('op_release',{owner:opOwner,seq:++opSeq});
}
function opEmergency(){opRelease();opSend('stop');opNotice('STOP diminta. Menunggu robot berhenti.');}
function opSpeed(v){$('op-speed').value=v;$('op-speed-value').textContent=v+'%';}
function opDraft(){return opFields.map((_,i)=>Number($('op-value-'+i).value));}
function opValidProfile(){return opFields.every((f,i)=>$('op-value-'+i).reportValidity());}
function opMarkDirty(){opDirty=true;$('op-profile-status').textContent='Ada perubahan, belum diterapkan';$('op-save').disabled=true;}
function opFill(values){values.forEach((v,i)=>{$('op-value-'+i).value=v;$('op-range-'+i).value=v;});}
async function opSelectProfile(){
  opRelease();
  if(opDirty && !confirm('Buang perubahan profil yang belum diterapkan?')){
    if(opData?.operator?.profile) $('op-profile').value=opData.operator.profile.id;return;
  }
  opRequested=Number($('op-profile').value);opDirty=false;opApplying=null;
  await opSend('op_profile',{id:opRequested});
}
async function opApplyProfile(){
  if(!opValidProfile()) return;
  opRelease();opApplying=opDraft();
  if(!await opSend('op_profile',{id:Number($('op-profile').value),values:opApplying}))opApplying=null;
}
function opQuickHeight(delta){
  if(!opHasControl() || !opData?.operator?.profile){opNotice('Aktifkan manual dan baca profil dahulu.');return;}
  const v=opDraft();v[0]=Math.max(0,Math.min(120,v[0]+delta));opFill(v);opMarkDirty();opApplyProfile();
}
function opLoadProfile(){opRelease();if(confirm('Muat ulang enam profil dari EEPROM? Perubahan RAM yang belum disimpan akan diganti.')){opDirty=false;opApplying=null;opSend('op_load');}}
function opResetProfile(){opRelease();if(confirm('Kembalikan profil ini ke bawaan firmware? EEPROM belum berubah sampai Simpan ditekan.')){opDirty=false;opApplying=null;opSend('op_defaults',{id:Number($('op-profile').value)});}}
function opSearch(query){
  const panel=document.querySelectorAll('.panel')[4];
  const term=query.toLowerCase();
  $('op-params').querySelectorAll('details').forEach(group=>{
    let found=false;
    group.querySelectorAll('.param-row').forEach(row=>{row.hidden=!!term&&!row.textContent.toLowerCase().includes(term);if(!row.hidden)found=true;});
    group.hidden=!found;if(term&&found)group.open=true;
  });
  document.querySelectorAll('#kalib>div,#kalib34>div,#trim>div').forEach(row=>{row.hidden=!!term&&!row.textContent.toLowerCase().includes(term);});
  panel.querySelectorAll(':scope > .card:not(.calibration-intro)').forEach(card=>{
    card.hidden=!!query&&!card.textContent.toLowerCase().includes(query.toLowerCase());
    if(query&&!card.hidden){const d=card.querySelector('details.card-details');if(d)d.open=true;}
  });
}
function opRenderParams(params){
  const keys=Object.keys(params), box=$('op-params');
  if(box.dataset.keys!==keys.join('|')){
    box.replaceChildren();let group='',container=box;
    const groups={gait:'Gerak dasar',wall:'Sensor dinding',heading:'Arah robot',stab:'Stabilisasi',condong:'Condong badan',head:'Kompas',arena:'Arena'};
    keys.forEach(name=>{
      const p=params[name], prefix=name.split('.')[0];
      if(group!==prefix){group=prefix;container=document.createElement('details');const h=document.createElement('summary');h.textContent=groups[group]||'Servo & parameter';container.append(h);container.open=group==='wall';box.append(container);}
      const row=document.createElement('div');row.className='param-row';
      const label=document.createElement('label');label.htmlFor='op-param-'+name;label.textContent=opHuman(name);
      const info=document.createElement('small');info.textContent=`${name} · ${p.min} sampai ${p.max} · ${['Langsung','Pilih ulang profil','Servo lemas','Belum dipakai'][p.effect]||''}`;label.append(info);
      const input=document.createElement('input');input.id=label.htmlFor;input.type='number';input.step='any';input.min=p.min;input.max=p.max;input.value=p.value;
      input.addEventListener('input',()=>{input.dataset.dirty='1';});
      const button=document.createElement('button');button.textContent='Terapkan';
      button.onclick=()=>{if(input.reportValidity())opSend('op_param',{name,value:Number(input.value)}).then(ok=>{if(ok) input.dataset.sent=input.value;});};
      row.append(label,input,button);container.append(row);
    });box.dataset.keys=keys.join('|');
  }
  keys.forEach(name=>{
    const input=$('op-param-'+name);if(!input)return;
    if(input.dataset.sent!==undefined && Number(input.dataset.sent)===params[name].value){delete input.dataset.dirty;delete input.dataset.sent;}
    if(input!==document.activeElement&&!input.dataset.dirty)input.value=params[name].value;
  });
}
function opReadableFields(){
  document.querySelectorAll('#kalib>div,#kalib34>div').forEach(row=>{
    if(row.dataset.labelled)return;const input=row.querySelector('input,select');if(!input)return;
    const name=input.id.replace(/^(kb_|k34_)/,'');
    Array.from(row.childNodes).filter(n=>n.nodeType===3).forEach(n=>n.remove());
    const label=document.createElement('label');label.htmlFor=input.id;label.textContent=opHuman(name);
    const code=document.createElement('span');code.className='field-code';code.textContent=name;label.append(code);row.prepend(label);row.dataset.labelled='1';
  });
  document.querySelectorAll('#trim>div').forEach(row=>{
    if(row.dataset.labelled)return;
    const text=Array.from(row.childNodes).find(n=>n.nodeType===3);if(!text)return;
    const label=document.createElement('label');label.className='trim-name';
    const input=row.querySelector('input');if(!input)return;label.htmlFor=input.id;input.type='number';input.min=-200;input.max=200;input.step=1;
    input.onchange=()=>{if(input.reportValidity())cmd('man','Yt'+input.id.slice(3)+' '+input.value);};
    const names=['Kanan depan','Kanan tengah','Kanan belakang','Kiri belakang','Kiri tengah','Kiri depan'];
    const raw=text.textContent.trim();label.textContent=raw.replace(/K([0-5])_/,(_,i)=>names[Number(i)]+' · ').replace('COXA','Coxa').replace('FEMUR','Femur').replace('TIBIA','Tibia')+' (µs)';
    text.replaceWith(label);row.dataset.labelled='1';
  });
}
window.operatorUpdate=function(d){
  opData=d;opSeen=performance.now();const s=d.operator||{};
  if(!opReadRequested&&d.port_ok){opReadRequested=true;opSend('op_read');}
  if(!d.port_ok){opReadRequested=false;if(opVector)opRelease();}
  $('op-enter').textContent=d.manual?'Keluar manual':'Aktifkan manual';
  $('op-unlock').hidden=!d.halt;
  $('op-enter').disabled=!d.port_ok||d.halt||!s.profile||(d.manual&&d.manual_owner!==opOwner);
  $('op-manual-state').textContent=d.halt?'STOP terkunci. Lepas STOP untuk memakai kontrol.':d.manual?(s.servo===true?'Manual aktif · tekan-tahan arah, lepas untuk berhenti.':'Manual aktif · tekan Berdiri untuk menyalakan servo sebelum bergerak.'):'Aktifkan manual untuk menghentikan otomasi dan mengambil kendali.';
  document.querySelectorAll('[data-drive]').forEach(b=>b.disabled=!opCanDrive());
  if(performance.now()>opLocalMessageUntil)$('op-notice').textContent=d.port_ok?(s.message||'Menunggu data robot.'):'Teensy belum tersambung. Kontrol gerak dinonaktifkan.';
  if(d.halt)$('op-notice').textContent='STOP terkunci. Gerak dinonaktifkan sampai Anda menekan Lepas STOP.';
  if(s.pending_age>5)$('op-notice').textContent='Belum ada konfirmasi firmware. Status simpan belum diketahui; periksa log sebelum mencoba lagi.';
  const p=s.profile;
  if(p && p.id>=0 && (opRequested===null||opRequested===p.id)){
    if(opApplying&&p.values.every((v,i)=>Math.abs(v-opApplying[i])<.011)){opDirty=false;opApplying=null;}
    const key=JSON.stringify(p);
    if(!opDirty&&(key!==opProfileKey||opRequested!==null)){$('op-profile').value=p.id;opFill(p.values);opProfileKey=key;opRequested=null;}
    $('op-step').textContent=p.values[0]+' mm';
    if(!opDirty)$('op-profile-status').textContent=s.saved?'EEPROM terverifikasi':((p.mask&(1<<p.id))?'Setelan khusus aktif · gunakan Simpan untuk memastikan tersimpan':'Bawaan firmware aktif');
  }
  const editable=opHasControl()&&!!p;
  $('op-profile').disabled=!editable;
  $('op-apply').disabled=!editable||!opDirty;
  $('op-save').disabled=!editable||opDirty||!!opApplying||!!(s.pending&&s.pending_age<5);
  $('op-profile-fields').querySelectorAll('input').forEach(el=>el.disabled=!editable);
  if(Object.keys(s.params||{}).length)opRenderParams(s.params);
  opReadableFields();
};
opFields.forEach((f,i)=>{
  const row=document.createElement('div');row.className='profile-row';
  row.innerHTML=`<label for="op-value-${i}">${f[0]}</label><input aria-label="${f[0]} slider" type=range id="op-range-${i}" min=${f[2]} max=${f[3]} step=${f[4]} disabled><input aria-label="${f[0]} dalam ${f[1]}" type=number id="op-value-${i}" min=${f[2]} max=${f[3]} step=any required disabled><small>${f[1]}</small>`;
  $('op-profile-fields').append(row);
  $('op-range-'+i).oninput=e=>{$('op-value-'+i).value=e.target.value;opMarkDirty();};
  $('op-value-'+i).oninput=e=>{$('op-range-'+i).value=e.target.value;opMarkDirty();};
});
$('op-speed').oninput=e=>opSpeed(e.target.value);
document.querySelectorAll('[data-drive]').forEach(button=>{
  button.disabled=true;
  button.addEventListener('pointerdown',e=>{if(e.button!==0)return;e.preventDefault();button.setPointerCapture(e.pointerId);button.classList.add('held');opDrive(button.dataset.drive.split(',').map(Number));});
  ['pointerup','pointercancel','lostpointercapture'].forEach(type=>button.addEventListener(type,()=>{if(opVector)opRelease();}));
  button.addEventListener('keydown',e=>{if((e.key===' '||e.key==='Enter')&&!e.repeat){e.preventDefault();opDrive(button.dataset.drive.split(',').map(Number));}});
  button.addEventListener('keyup',e=>{if(e.key===' '||e.key==='Enter'){e.preventDefault();opRelease();}});
  button.addEventListener('blur',()=>{if(opVector)opRelease();});
});
function opKeyVector(){const has=(...k)=>k.some(v=>opKeys.has(v));return [(has('w','arrowup')?1:0)-(has('s','arrowdown')?1:0),(has('d','arrowright')?1:0)-(has('a','arrowleft')?1:0),(has('q')?1:0)-(has('e')?1:0)];}
document.addEventListener('keydown',e=>{
  if(e.key==='Escape'){e.preventDefault();opEmergency();return;}
  if(!$('op-keyboard').checked||/INPUT|SELECT|TEXTAREA|BUTTON|SUMMARY/.test(e.target.tagName)||e.ctrlKey||e.altKey||e.metaKey)return;
  const key=e.key.toLowerCase();if(!['w','a','s','d','q','e','arrowup','arrowdown','arrowleft','arrowright'].includes(key))return;
  e.preventDefault();if(e.repeat)return;opKeys.add(key);opDrive(opKeyVector());
});
document.addEventListener('keyup',e=>{if(!opKeys.delete(e.key.toLowerCase()))return;const v=opKeyVector();if(v.some(Boolean))opVector=v;else opRelease();});
document.addEventListener('focusin',e=>{if(/INPUT|SELECT|TEXTAREA/.test(e.target.tagName)&&opVector)opRelease();});
window.addEventListener('blur',()=>{if(opVector)opRelease();});
document.addEventListener('visibilitychange',()=>{if(document.hidden)opRelease();});
window.addEventListener('pagehide',()=>{if(opVector)navigator.sendBeacon('/cmd?k=op_release&v='+encodeURIComponent(JSON.stringify({owner:opOwner,seq:++opSeq})));});
$('op-keyboard').onchange=()=>opRelease();
document.querySelectorAll('#tabs button').forEach(b=>b.addEventListener('click',()=>{opRelease();$('op-keyboard').checked=false;}));
setInterval(()=>{if(opVector&&performance.now()-opSeen>1200)opRelease();else opTick();},180);

// Keep diagnostics available, but out of the movement and tuning workflow.
document.querySelectorAll('.card').forEach(card=>{
  if(card.matches('.operator-main,.profile-editor,.calibration-intro,.firmware-calibration'))return;
  const paragraphs=Array.from(card.querySelectorAll(':scope > p')).filter(p=>p.textContent.length>200);
  if(paragraphs.length){const details=document.createElement('details');details.className='inline-help';const summary=document.createElement('summary');summary.textContent='Petunjuk & penjelasan';details.append(summary);paragraphs.forEach(p=>details.append(p));card.append(details);}
});
const opPanels=document.querySelectorAll('.panel');
opPanels[0].querySelectorAll(':scope > .card:not(.operator-main):not(.profile-editor)').forEach(card=>opFold(card));
opPanels[4].querySelectorAll(':scope > .card:not(.calibration-intro):not(.firmware-calibration)').forEach(card=>opFold(card));
document.querySelectorAll('.wrap>div:first-child>.card').forEach(card=>opFold(card));
function opFold(card){
  const heading=card.querySelector(':scope > h2');if(!heading)return;
  const details=document.createElement('details');details.className='card-details';
  const summary=document.createElement('summary');summary.textContent=heading.childNodes[0].textContent.trim();
  heading.querySelectorAll('button').forEach(b=>card.append(b));heading.remove();
  const content=Array.from(card.childNodes);details.append(summary,...content);card.append(details);
}
document.querySelectorAll('[data-tip]').forEach(el=>el.title=el.dataset.tip);
