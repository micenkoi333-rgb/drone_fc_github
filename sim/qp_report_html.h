// qp_report_html.h — звіт про політ одним HTML-файлом: підсумок, графіки всіх величин, карта траєкторії (носій + субдрони),
// події й кнопки «скачати CSV». Файл самодостатній (без інтернету): дані вбудовані як CSV, графіки малює вбудований JavaScript.
#pragma once
#include <cstdio>
#include <string>
#include "qp_flight.h"

namespace sim {

namespace detail {
inline std::string htmlEscape(const std::string& s) {
    std::string o; o.reserve(s.size());
    for (char c : s) {
        if (c == '<') o += "&lt;"; else if (c == '>') o += "&gt;"; else if (c == '&') o += "&amp;"; else o += c;
    }
    return o;
}
}  // namespace detail

inline bool writeHtmlReport(const QpFlightRig& rig, const std::string& path, const std::string& title) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::string o;
    o.reserve(rig.log.size() * 300 + 60000);
    o += "<!doctype html>\n<html lang=\"uk\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n<title>";
    o += detail::htmlEscape(title);
    o += "</title>\n";
    o += R"HTML(<style>
:root{--bg:#f6f7f9;--card:#fff;--fg:#1d2330;--mut:#6b7385;--grid:#e3e6ec;--axis:#9aa1b0;--acc:#2f6fe4}
@media (prefers-color-scheme:dark){:root{--bg:#12151b;--card:#1b1f27;--fg:#e6e9ef;--mut:#8d95a6;--grid:#2a303b;--axis:#5d6576;--acc:#6aa0ff}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
header{padding:18px 20px 8px}h1{margin:0 0 4px;font-size:22px}h2{font-size:16px;margin:0 0 8px}
.sub{color:var(--mut)}.wrap{padding:0 20px 30px;max-width:1700px;margin:0 auto}
.bar{display:flex;flex-wrap:wrap;gap:8px;margin:10px 0 16px}
button{background:var(--acc);color:#fff;border:0;border-radius:8px;padding:8px 14px;font:inherit;cursor:pointer}button.sec{background:transparent;color:var(--fg);border:1px solid var(--grid)}
.card{background:var(--card);border:1px solid var(--grid);border-radius:12px;padding:14px;margin-bottom:14px}
.grid2{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:14px}
@media (max-width:1100px){.grid2{grid-template-columns:minmax(0,1fr)}}
pre{margin:0;white-space:pre-wrap;font:13px/1.5 ui-monospace,Consolas,monospace}
.charts{display:grid;grid-template-columns:repeat(auto-fill,minmax(520px,1fr));gap:14px}
@media (max-width:600px){.charts{grid-template-columns:minmax(0,1fr)}}
.ch{background:var(--card);border:1px solid var(--grid);border-radius:12px;padding:10px 12px 6px}
.ch .t{font-weight:600;margin-bottom:2px}.ch .lg{font-size:12px;color:var(--mut);min-height:18px;display:flex;flex-wrap:wrap;gap:10px}
.ch .lg i{display:inline-block;width:10px;height:3px;border-radius:2px;margin-right:4px;vertical-align:middle}
.cv{position:relative;width:100%;height:220px}.cv canvas{position:absolute;inset:0;width:100%;height:100%}
#map{height:560px}.phases{display:flex;flex-wrap:wrap;gap:12px;font-size:12px;color:var(--mut);margin:6px 0 10px}.phases i{display:inline-block;width:12px;height:12px;border-radius:3px;margin-right:4px;vertical-align:-2px}
table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:5px 8px;border-bottom:1px solid var(--grid);text-align:left}
.hint{font-size:12px;color:var(--mut)}
</style></head><body>
)HTML";
    o += "<header class=\"wrap\" style=\"padding-top:18px\"><h1>" + detail::htmlEscape(title) + "</h1><div class=\"sub\">Конвертоплан (4 поворотні гвинти, V-хвіст) — симулятор drone_fc</div>\n";
    o += R"HTML(<div class="bar"><button onclick="dl('csv','_full.csv')">Скачати CSV — усі дані польоту</button><button class="sec" onclick="dl('txt','.txt')">Звіт .txt</button><button class="sec" onclick="resetZoom()">Скинути масштаб</button></div>
<div class="hint">Графіки: наведіть — значення в легенді й точка на карті; виділіть ділянку мишею — збільшити час; подвійний клік — увесь політ. Карта: колесо — масштаб, перетягування — зсув.</div></header>
<div class="wrap">
<div class="grid2"><div class="card"><h2>Підсумок</h2><pre id="txt">)HTML";
    o += detail::htmlEscape(rig.reportText());
    o += R"HTML(</pre></div><div class="card"><h2>Траєкторія (вид зверху)</h2><div class="phases" id="phl"></div><div class="cv" id="map"><canvas id="mapc"></canvas></div></div></div>
<h2 style="margin:6px 0 10px">Графіки</h2><div class="phases" id="phl2"></div><div class="charts" id="charts"></div>
</div>
<script type="text/plain" id="csv">)HTML";
    o += rig.csvText();
    o += "</script>\n<script>\n";
    o += "const FILEBASE=" + std::string("\"") + title + "\";\n";
    o += R"JS(
const PH=['на землі','зліт','висіння','перехід у літак','літак','перехід у коптер','посадка','сів','посадка літаком','ручне'];
const PC=['#888','#e8a33d','#d06bd6','#4fb3d9','#3a7be0','#22b8a2','#e05a5a','#888','#b5652b','#9b9b2f'];
function parse(id){const L=document.getElementById(id).textContent.trim().split('\n');const h=L[0].split(',');const D={};h.forEach(k=>D[k]=new Float64Array(L.length-1));
 for(let i=1;i<L.length;i++){const v=L[i].split(',');for(let j=0;j<h.length;j++)D[h[j]][i-1]=+v[j];}D.n=L.length-1;return D;}
const D=parse('csv');const N=D.n;
// похідні величини
const R2D=180/Math.PI;function mk(f){const a=new Float64Array(N);for(let i=0;i<N;i++)a[i]=f(i);return a;}
D.gs=mk(i=>Math.hypot(D.vx[i],D.vy[i]));D.ws=mk(i=>Math.hypot(D.wind_x[i],D.wind_y[i]));D.rolld=mk(i=>D.roll[i]*R2D);D.pitchd=mk(i=>-D.pitch_down[i]*R2D);
D.yawd=mk(i=>((D.yaw[i]*R2D)%360+360)%360);D.socp=mk(i=>D.soc[i]*100);D.home=mk(i=>Math.hypot(D.x[i],D.y[i]));D.wdir=mk(i=>((Math.atan2(-D.wind_x[i],-D.wind_y[i])*R2D)+360)%360);
for(const k of['duty1','duty2','duty3','duty4'])D[k+'p']=mk(i=>D[k][i]*100);
D.pd=mk(i=>D.p_rad_s[i]*R2D);D.qd=mk(i=>D.q_rad_s[i]*R2D);D.rd=mk(i=>D.r_rad_s[i]*R2D);D.wvp=mk(i=>D.vtol_weight[i]*100);D.limp=mk(i=>D.current_limit[i]*100);
D.whu=new Float64Array(N);for(let i=1;i<N;i++)D.whu[i]=D.whu[i-1]+D.power_w[i]*(D.t[i]-D.t[i-1])/3600;
const T0=N?D.t[0]:0,T1=N?D.t[N-1]:1;let X0=T0,X1=T1,CUR=null;
const CH=[
 ['Висота, м',[['z','висота','#3a7be0']]],
 ['Вертикальна швидкість, м/с',[['vz','Vz','#e05a5a']]],
 ['Швидкість, м/с',[['airspeed','повітряна','#3a7be0'],['gs','земна','#22b8a2'],['ws','вітер','#e8a33d']]],
 ['Відстань від дому, м',[['home','відстань','#9b6bd6']]],
 ['Крен і тангаж, °',[['rolld','крен','#3a7be0'],['pitchd','тангаж (ніс угору +)','#e05a5a']]],
 ['Курс, °',[['yawd','курс','#3a7be0'],['wdir','звідки вітер','#e8a33d']]],
 ['Кутові швидкості, °/с',[['pd','p (крен)','#3a7be0'],['qd','q (тангаж)','#e05a5a'],['rd','r (рискання)','#22b8a2']]],
 ['Вітер біля апарата, м/с',[['wind_x','схід (X)','#3a7be0'],['wind_y','північ (Y)','#22b8a2'],['wind_z','вертикальний','#e05a5a']]],
 ['Напруга батареї, В',[['volts','напруга','#3a7be0']]],
 ['Струм, А',[['amps','струм','#e05a5a']]],
 ['Потужність, Вт',[['power_w','потужність','#e8a33d']]],
 ['Заряд батареї, % / витрачено, Вт·год',[['socp','заряд %','#22b8a2'],['whu','Вт·год','#9b6bd6']]],
 ['Температура батареї, °C',[['batt_temp_c','температура','#e05a5a']]],
 ['Оберти моторів, об/с',[['rps1','М1','#3a7be0'],['rps2','М2','#e05a5a'],['rps3','М3','#22b8a2'],['rps4','М4','#e8a33d']]],
 ['Газ моторів, %',[['duty1p','М1','#3a7be0'],['duty2p','М2','#e05a5a'],['duty3p','М3','#22b8a2'],['duty4p','М4','#e8a33d']]],
 ['Нахил гондол, ° (0 — вгору, 90 — уперед)',[['tilt1_deg','М1 перед. прав.','#3a7be0'],['tilt2_deg','М2 задн. лів.','#e05a5a'],['tilt3_deg','М3 перед. лів.','#22b8a2'],['tilt4_deg','М4 задн. прав.','#e8a33d']]],
 ['Керма (−1…1)',[['aileron','елерони','#3a7be0'],['elevator','руль висоти','#e05a5a'],['rudder','руль напрямку','#22b8a2']]],
 ['Коптер/літак і обмежувач струму, %',[['wvp','вага коптера','#9b6bd6'],['limp','дозволений струм','#e05a5a']]],
];
function css(v){return getComputedStyle(document.documentElement).getPropertyValue(v).trim();}
function nice(lo,hi,n){const r=(hi-lo)||1;const s=Math.pow(10,Math.floor(Math.log10(r/n)));const st=[1,2,5,10].map(m=>m*s).find(x=>r/x<=n)||10*s;const a=[];for(let v=Math.ceil(lo/st)*st;v<=hi+1e-9;v+=st)a.push(+v.toFixed(10));return a;}
function fmt(v){const a=Math.abs(v);return a>=100?v.toFixed(0):a>=10?v.toFixed(1):v.toFixed(2);}
function idxAt(t){let lo=0,hi=N-1;while(hi-lo>1){const m=(lo+hi)>>1;if(D.t[m]<t)lo=m;else hi=m;}return (Math.abs(D.t[hi]-t)<Math.abs(D.t[lo]-t))?hi:lo;}
function setup(cv){const r=cv.getBoundingClientRect(),d=devicePixelRatio||1;cv.width=Math.max(1,r.width*d);cv.height=Math.max(1,r.height*d);const g=cv.getContext('2d');g.setTransform(d,0,0,d,0,0);return [g,r.width,r.height];}
const charts=[];
function makeCharts(){const host=document.getElementById('charts');
 const list=CH.slice();
 for(const [title,ser] of list){const el=document.createElement('div');el.className='ch';el.innerHTML='<div class="t"></div><div class="lg"></div><div class="cv"><canvas></canvas><canvas></canvas></div>';
  el.querySelector('.t').textContent=title;host.appendChild(el);const [b,ov]=el.querySelectorAll('canvas');const c={el,b,ov,ser,lg:el.querySelector('.lg')};charts.push(c);bind(c);}
}
function seriesData(key){ return [D.t,D[key]];}
const PADL=48,PADR=10,PADT=8,PADB=20;
function draw(c){const [g,w,h]=setup(c.b);c.w=w;c.h=h;g.clearRect(0,0,w,h);
 let lo=Infinity,hi=-Infinity;const data=c.ser.map(s=>seriesData(s[0]));
 for(const [xs,ys] of data)for(let i=0;i<xs.length;i++)if(xs[i]>=X0&&xs[i]<=X1){const v=ys[i];if(v<lo)lo=v;if(v>hi)hi=v;}
 if(!isFinite(lo)){lo=0;hi=1;}if(hi-lo<1e-6){hi+=0.5;lo-=0.5;}const pad=(hi-lo)*0.06;lo-=pad;hi+=pad;c.lo=lo;c.hi=hi;
 const X=t=>PADL+(t-X0)/(X1-X0)*(w-PADL-PADR),Y=v=>PADT+(hi-v)/(hi-lo)*(h-PADT-PADB);c.X=X;c.Y=Y;
 // смуга режимів
 if(N){let s=0;for(let i=1;i<=N;i++){if(i==N||D.phase[i]!=D.phase[s]){const a=Math.max(X(D.t[s]),PADL),b=Math.min(X(D.t[Math.min(i,N-1)]),w-PADR);if(b>a){g.fillStyle=PC[D.phase[s]]||'#888';g.globalAlpha=0.08;g.fillRect(a,PADT,b-a,h-PADT-PADB);g.globalAlpha=0.9;g.fillRect(a,h-PADB+2,b-a,3);g.globalAlpha=1;}s=i;}}}
 g.strokeStyle=css('--grid');g.fillStyle=css('--mut');g.font='11px system-ui';g.lineWidth=1;
 for(const v of nice(lo,hi,5)){const y=Y(v);g.beginPath();g.moveTo(PADL,y);g.lineTo(w-PADR,y);g.stroke();g.fillText(fmt(v),4,y+4);}
 for(const t of nice(X0,X1,8)){const x=X(t);g.beginPath();g.moveTo(x,PADT);g.lineTo(x,h-PADB);g.stroke();g.fillText(t.toFixed(0)+' с',x-10,h-4);}
 g.save();g.beginPath();g.rect(PADL,PADT-2,w-PADL-PADR,h-PADT-PADB+2);g.clip();
 data.forEach(([xs,ys],k)=>{g.strokeStyle=c.ser[k][2];g.lineWidth=1.4;g.beginPath();let st=false;const step=Math.max(1,Math.floor(xs.length/((w-PADL)*3)*((X1-X0)/(T1-T0||1))));
  for(let i=0;i<xs.length;i++){if(xs[i]<X0-1||xs[i]>X1+1)continue;const x=X(xs[i]),y=Y(ys[i]);if(!st){g.moveTo(x,y);st=true;}else g.lineTo(x,y);}g.stroke();});
 g.restore();c.data=data;overlay(c);}
function overlay(c){const [g,w,h]=setup(c.ov);g.clearRect(0,0,w,h);let html='';
 if(c.sel){g.fillStyle=css('--acc');g.globalAlpha=0.15;g.fillRect(Math.min(c.sel[0],c.sel[1]),PADT,Math.abs(c.sel[1]-c.sel[0]),h-PADT-PADB);g.globalAlpha=1;}
 if(CUR!=null&&c.X){const x=c.X(CUR);g.strokeStyle=css('--fg');g.globalAlpha=0.5;g.beginPath();g.moveTo(x,PADT);g.lineTo(x,h-PADB);g.stroke();g.globalAlpha=1;}
 c.ser.forEach((s,k)=>{let val='';if(CUR!=null&&c.data){const [xs,ys]=c.data[k];let best=-1,bd=1e9;
   if(xs.length){let lo=0,hi=xs.length-1;while(hi-lo>1){const m=(lo+hi)>>1;if(xs[m]<CUR)lo=m;else hi=m;}best=Math.abs(xs[hi]-CUR)<Math.abs(xs[lo]-CUR)?hi:lo;bd=Math.abs(xs[best]-CUR);}
   if(best>=0&&bd<2){val=': <b>'+fmt(ys[best])+'</b>';const y=c.Y(ys[best]);g.fillStyle=s[2];g.beginPath();g.arc(c.X(CUR),y,3,0,7);g.fill();}}
  html+='<span><i style="background:'+s[2]+'"></i>'+s[1]+val+'</span>';});
 if(CUR!=null&&N){const i=idxAt(CUR);html='<span>t = <b>'+D.t[i].toFixed(1)+' с</b>, '+PH[D.phase[i]]+'</span>'+html;}
 c.lg.innerHTML=html;}
function bind(c){const cv=c.ov;let down=null;
 cv.addEventListener('mousemove',e=>{const r=cv.getBoundingClientRect(),x=e.clientX-r.left;if(!c.X)return;const t=X0+(x-PADL)/(c.w-PADL-PADR)*(X1-X0);CUR=Math.max(X0,Math.min(X1,t));if(down!=null)c.sel=[down,x];refreshOverlays();});
 cv.addEventListener('mouseleave',()=>{CUR=null;down=null;c.sel=null;refreshOverlays();});
 cv.addEventListener('mousedown',e=>{const r=cv.getBoundingClientRect();down=e.clientX-r.left;});
 cv.addEventListener('mouseup',e=>{const r=cv.getBoundingClientRect(),x=e.clientX-r.left;if(down!=null&&Math.abs(x-down)>6){const ta=X0+(Math.min(x,down)-PADL)/(c.w-PADL-PADR)*(X1-X0),tb=X0+(Math.max(x,down)-PADL)/(c.w-PADL-PADR)*(X1-X0);X0=Math.max(T0,ta);X1=Math.min(T1,tb);if(X1-X0<1)X1=X0+1;down=null;c.sel=null;redrawAll();}down=null;c.sel=null;});
 cv.addEventListener('dblclick',resetZoom);}
function resetZoom(){X0=T0;X1=T1;redrawAll();}
function refreshOverlays(){charts.forEach(overlay);drawMapOverlay();}
// --- карта
let MV=null,mapDrag=null;
function mapFit(){let x0=0,x1=0,y0=0,y1=0;const ext=(x,y)=>{x0=Math.min(x0,x);x1=Math.max(x1,x);y0=Math.min(y0,y);y1=Math.max(y1,y);};
 for(let i=0;i<N;i++)ext(D.x[i],D.y[i]);
 const cv=document.getElementById('mapc'),r=cv.getBoundingClientRect();const s=Math.min((r.width-40)/Math.max(x1-x0,20),(r.height-40)/Math.max(y1-y0,20));MV={s,cx:(x0+x1)/2,cy:(y0+y1)/2};}
function MX(x){const cv=document.getElementById('mapc');return cv.clientWidth/2+(x-MV.cx)*MV.s;}function MY(y){const cv=document.getElementById('mapc');return cv.clientHeight/2-(y-MV.cy)*MV.s;}
let mapImg=null;
function drawMap(){const cv=document.getElementById('mapc');const [g,w,h]=setup(cv);if(!MV)mapFit();g.clearRect(0,0,w,h);
 // сітка з кроком
 const span=w/MV.s;let st=Math.pow(10,Math.floor(Math.log10(span/6)));if(span/st>12)st*=5;else if(span/st>6)st*=2;
 g.strokeStyle=css('--grid');g.lineWidth=1;const xa=MV.cx-w/2/MV.s,xb=MV.cx+w/2/MV.s,ya=MV.cy-h/2/MV.s,yb=MV.cy+h/2/MV.s;
 for(let x=Math.ceil(xa/st)*st;x<xb;x+=st){g.beginPath();g.moveTo(MX(x),0);g.lineTo(MX(x),h);g.stroke();}
 for(let y=Math.ceil(ya/st)*st;y<yb;y+=st){g.beginPath();g.moveTo(0,MY(y));g.lineTo(w,MY(y));g.stroke();}
 g.fillStyle=css('--mut');g.font='11px system-ui';g.fillText('сітка '+st+' м   ↑ північ (Y), → схід (X)',8,h-8);
 // носій: колір — режим
 g.lineWidth=2.2;for(let i=1;i<N;i++){g.strokeStyle=PC[D.phase[i]]||'#888';g.beginPath();g.moveTo(MX(D.x[i-1]),MY(D.y[i-1]));g.lineTo(MX(D.x[i]),MY(D.y[i]));g.stroke();}
 g.fillStyle=css('--fg');g.beginPath();g.moveTo(MX(0),MY(0)-7);g.lineTo(MX(0)+6,MY(0)+5);g.lineTo(MX(0)-6,MY(0)+5);g.closePath();g.fill();g.fillText('дім',MX(0)+8,MY(0)+4);
 drawMapOverlay();}
function drawMapOverlay(){const cv=document.getElementById('mapc');if(!MV)return;let ov=document.getElementById('mapo');
 if(!ov){ov=document.createElement('canvas');ov.id='mapo';cv.parentNode.appendChild(ov);bindMap(ov);}const [g,w,h]=setup(ov);g.clearRect(0,0,w,h);
 if(CUR!=null&&N){const i=idxAt(CUR);g.fillStyle=css('--fg');g.strokeStyle=css('--card');g.lineWidth=2;g.beginPath();g.arc(MX(D.x[i]),MY(D.y[i]),6,0,7);g.fill();g.stroke();
  g.font='12px system-ui';g.fillText(D.t[i].toFixed(0)+' с, '+D.z[i].toFixed(0)+' м',MX(D.x[i])+9,MY(D.y[i])-9);
}
function bindMap(ov){ov.addEventListener('wheel',e=>{e.preventDefault();const r=ov.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top;
  const wx=MV.cx+(mx-r.width/2)/MV.s,wy=MV.cy-(my-r.height/2)/MV.s;const k=e.deltaY<0?1.25:0.8;MV.s*=k;MV.cx=wx-(mx-r.width/2)/MV.s;MV.cy=wy+(my-r.height/2)/MV.s;drawMap();},{passive:false});
 ov.addEventListener('mousedown',e=>{mapDrag=[e.clientX,e.clientY,MV.cx,MV.cy];});window.addEventListener('mouseup',()=>mapDrag=null);
 ov.addEventListener('mousemove',e=>{if(!mapDrag)return;MV.cx=mapDrag[2]-(e.clientX-mapDrag[0])/MV.s;MV.cy=mapDrag[3]+(e.clientY-mapDrag[1])/MV.s;drawMap();});
 ov.addEventListener('dblclick',()=>{mapFit();drawMap();});}
function legendPhases(){const used=new Set();for(let i=0;i<N;i++)used.add(D.phase[i]);let h='';[...used].sort((a,b)=>a-b).forEach(p=>h+='<span><i style="background:'+PC[p]+'"></i>'+PH[p]+'</span>');
 let hs='';
 document.getElementById('phl').innerHTML=h+hs;document.getElementById('phl2').innerHTML='Кольорова смуга під графіками — режим польоту: '+h;}
function redrawAll(){charts.forEach(draw);}
function dl(id,suf){const txt=document.getElementById(id).textContent;const b=new Blob([id=='txt'?'﻿'+txt:txt],{type:id=='txt'?'text/plain;charset=utf-8':'text/csv;charset=utf-8'});
 const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download=FILEBASE+suf;document.body.appendChild(a);a.click();setTimeout(()=>{URL.revokeObjectURL(a.href);a.remove();},500);}
makeCharts();legendPhases();redrawAll();drawMap();
let rt=null;window.addEventListener('resize',()=>{clearTimeout(rt);rt=setTimeout(()=>{redrawAll();drawMap();},150);});
matchMedia('(prefers-color-scheme: dark)').addEventListener('change',()=>{redrawAll();drawMap();});
</script></body></html>
)JS";
    const bool ok = std::fwrite(o.data(), 1, o.size(), f) == o.size();
    std::fclose(f);
    return ok;
}

}  // namespace sim
