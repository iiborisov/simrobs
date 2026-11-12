"""Встроить CMU/KaTeX и расширить существующие интерактивные презентации.
Запуск: python upgrade_presentations.py (нужны lxml; локальные vendor-файлы).
"""
from pathlib import Path
from lxml import html, etree
import base64, json, re
from lecture_expansions import EXPANSIONS

BASE=Path(__file__).resolve().parent
ROOT=BASE.parent

def b64(path):return base64.b64encode(path.read_bytes()).decode()
def font_css():
    out=[]
    for stem,weight,style,family in [('cmunrm',400,'normal','CMU Serif'),('cmunbx',700,'normal','CMU Serif'),('cmunti',400,'italic','CMU Serif'),('cmunbi',700,'italic','CMU Serif'),('cmuntt',400,'normal','CMU Typewriter')]:
        out.append('@font-face{font-family:"'+family+'";font-weight:'+str(weight)+';font-style:'+style+';font-display:block;src:url(data:font/woff;base64,'+b64(BASE/'cmu'/(stem+'.woff'))+') format("woff")}')
    css=(BASE/'katex-0.16.22/dist/katex.min.css').read_text()
    def source(m):
        match=re.search(r'url\((?:["\']?)([^)"\']+\.woff2)',m.group(1))
        if not match:return m.group(0)
        return 'src:url(data:font/woff2;base64,'+b64(BASE/'katex-0.16.22/dist'/match.group(1))+') format("woff2");'
    css=re.sub(r'src:([^;]+);',source,css)
    return '\n'.join(out)+'\n'+css

FORMULAS={
'lecture_1_modelling_n_simulation':{
2:[r'm\ddot x=u-b\dot x-kx-mg'],
3:[r'x_{\rm eq}=-\frac{mg}{k},\qquad \xi=x-x_{\rm eq}',r'm\ddot\xi+b\dot\xi+k\xi=u'],
4:[r'T=\frac12m\dot x^2,\qquad V=\frac12kx^2+mgx',r'L=T-V',r'\frac{d}{dt}\frac{\partial L}{\partial\dot x}-\frac{\partial L}{\partial x}=Q'],
5:[r'y=\begin{bmatrix}x\\v\end{bmatrix},\qquad \dot y=\begin{bmatrix}v\\\dfrac{u-bv-kx-mg}{m}\end{bmatrix}',r'\dot y=f(t,y),\qquad y(0)=y_0'],
6:[r'\omega_n=\sqrt{\frac{k}{m}},\qquad\zeta=\frac{b}{2\sqrt{mk}}'],
7:[r'y_{n+1}=y_n+hf(t_n,y_n)',r'y_{n+1}=y_n+hf(t_{n+1},y_{n+1})',r'y_{n+1}=y_n+\frac h6(k_1+2k_2+2k_3+k_4)'],
9:[r'E_{n+1}=\left(1+h^2\frac{k}{m}\right)E_n',r'E_{n+1}=\frac{E_n}{1+h^2k/m}'],
10:[r'\begin{aligned}F(z)&=z-y_n-hf(t_{n+1},z)=0,\\J_F(z)\Delta z&=-F(z),\\z&\leftarrow z+\Delta z.\end{aligned}'],
11:[r'\begin{aligned}\dot y&=\lambda y,\\y_{n+1}&=R(h\lambda)y_n,\\|R(h\lambda)|&<1.\end{aligned}'],
12:[r'\dot x=v,\qquad\dot v=\frac{u-bv-kx-mg}{m}'],
},
'lecture_2_2R_robot':{
2:[r'\begin{aligned}x&=l_1\cos q_1+l_2\cos(q_1+q_2),\\z&=l_1\sin q_1+l_2\sin(q_1+q_2).\end{aligned}'],
3:[r'c_2=\frac{x^2+z^2-l_1^2-l_2^2}{2l_1l_2}'],
4:[r'\dot p=J(q)\dot q',r'\det J=l_1l_2\sin q_2'],
5:[r'L=T-V',r'\frac{d}{dt}\frac{\partial L}{\partial\dot q_i}-\frac{\partial L}{\partial q_i}=\tau_i-b_i\dot q_i'],
6:[r'M(q)\ddot q+c(q,\dot q)+G(q)+D\dot q=\tau',r'M(q)=\begin{bmatrix}a+2\beta\cos q_2&d+\beta\cos q_2\\d+\beta\cos q_2&d\end{bmatrix}'],
7:[r'M(q)\ddot q=\tau-c-G-D\dot q'],
8:[r'\begin{aligned}x_d&=0.4+A\sin\theta,\\z_d&=0.2+B\sin2\theta.\end{aligned}'],
9:[r'\tau=K_p(q_d-q)+K_d(\dot q_d-\dot q)+G(q)'],
},
'lecture_3_bond_graphs':{
1:[r'P=e f'],
4:[r'\dot q=f,\qquad e=\frac{\partial H}{\partial q}=\frac q C',r'\dot p=e,\qquad f=\frac{\partial H}{\partial p}=\frac p I'],
6:[r'\begin{aligned}f_s=f_C=f_R=f_I&=v,\\e_s&=e_C+e_R+e_I.\end{aligned}'],
7:[r'\dot x=\frac p m,\qquad\dot p=u-mg-kx-\frac b m p',r'\begin{aligned}H&=\frac{p^2}{2m}+\frac{kx^2}{2},\\\dot H&=(u-mg)v-bv^2.\end{aligned}'],
8:[r'\begin{aligned}i&=\frac\phi L,\qquad U_C=\frac{q_e}{C},\\\dot q_e&=i,\\\dot\phi&=U-Ri-\frac{q_e}{C}.\end{aligned}'],
9:[r'e_2=ne_1,\quad f_1=nf_2,\quad e_1f_1=e_2f_2'],
10:[r'e_{\rm back}=K\omega,\quad\tau_{\rm em}=Ki,\quad e_{\rm back}i=\tau_{\rm em}\omega'],
11:[r'J_{\rm eq}=J_m+\frac{J_l}{n^2},\qquad b_{\rm eq}=b_m+\frac{b_l}{n^2}',r'\begin{aligned}\dot\phi&=U-R\frac\phi L-K\frac{p_m}{J_{\rm eq}},\\\dot p_m&=K\frac\phi L-b_{\rm eq}\frac{p_m}{J_{\rm eq}}-\frac k n\theta_l,\\\dot\theta_l&=\frac{p_m}{nJ_{\rm eq}}.\end{aligned}'],
12:[r'\omega_m\to0,\quad i\to\frac U R,\quad\theta_l\to\frac{nKU}{Rk}'],
13:[r'\begin{aligned}\dot H&=Ui-Ri^2-b_{\rm eq}\omega_m^2,\\r_H&=H-H_0-W_{\rm in}+E_{\rm diss}.\end{aligned}'],
}}

STYLE='''
/* CMU и увеличенный набор для 90-минутных лекций. */
html,body{height:100%;overflow:hidden}body{font-family:"CMU Serif",serif!important;font-size:26px!important;line-height:1.4}
body,h1,h2,h3,p,li,table,button,select,label,.brand,.course,.eyebrow,.note,.small,.caption,.counter,.hint{font-family:"CMU Serif",serif!important}
header{height:66px!important}footer{height:58px!important}.brand{font-size:25px!important}.course{font-size:16px!important}
.slide{height:calc(100dvh - 124px);min-height:0!important;padding:26px 4vw 32px!important;overflow-y:auto;scrollbar-width:thin;scrollbar-color:#c4d9e8 #fff}
.slide.active{display:block}.slide .split{gap:3vw}.slide .split>*{min-width:0}
h1{font-size:clamp(62px,5vw,86px)!important;line-height:1.06!important;letter-spacing:0!important}h2{font-size:46px!important;line-height:1.12!important;margin-bottom:22px!important}h3{font-size:29px!important}
p{margin-bottom:16px!important}p.lead{font-size:29px!important;line-height:1.38}.small,.note{font-size:21px!important;line-height:1.38!important}.note{margin-top:18px!important;padding-top:14px!important}
.eyebrow{font-size:18px!important;margin-bottom:16px!important}.stage .caption{font-size:18px!important;line-height:1.35}.stage .label{font:16px "CMU Serif",serif!important}
.equation{font-family:"CMU Serif",serif!important;font-size:32px!important;line-height:1.45;overflow-x:auto;padding:16px 10px!important;margin:20px 0!important}
.katex{font-size:1.08em!important}.katex-display{margin:.2em 0!important}.katex .cyrillic_fallback{font-family:"CMU Serif",serif!important}
.methods .equation{font-size:28px!important}.math-tex.matrix{white-space:normal;font-size:32px!important}.math-inline{font-size:1em}.math-inline .katex{font-size:1.05em!important}
.control label{font-size:22px!important}.control output{font-size:22px}.controls{padding:18px!important;gap:12px 18px!important}
button,select{font-size:23px!important}.tools button{font-size:17px!important}.task{padding:20px!important}.task strong{font-size:29px!important}.task span{font-size:23px!important}.task em{font-size:26px}.task-detail{font-size:26px!important}
.table{font-size:24px!important}.table th{font-size:21px!important}.table td,.table th{padding:10px!important}
.metric{font-size:40px!important}.metric small{font-size:20px!important}.matrix{font-size:28px!important}.legend,.chip,.pill{font-size:19px!important}.legend{gap:16px}.feedback{font-size:26px!important;min-height:80px!important}
pre,code{font-family:"CMU Typewriter",monospace!important}pre{font-size:20px!important;line-height:1.45!important}code{font-size:.9em}
.pipeline b{font-size:23px!important}.pipeline{gap:12px}.references a{font-size:24px!important;margin:13px 0!important}.references p{font-size:22px!important}.overview-list button{font-size:23px!important}.overview-list small{font-size:18px}
.counter{font-size:17px!important;min-width:74px}.hint{font-size:15px!important}.nav button{font-size:24px!important}.time-label{font:17px "CMU Serif",serif;color:#7b263d;white-space:nowrap}.progress{min-width:80px}
.prose{max-width:1250px}.prose li,.book-list li{margin:10px 0}.discussion{margin-top:24px;border-top:1px solid #c4d9e8;padding-top:18px;max-width:1350px}.discussion h3{color:#7b263d}.discussion details{margin-top:14px;background:#f3f8fc;padding:14px 18px}.discussion summary{cursor:pointer;color:#7b263d;font-size:23px}.discussion details>div{padding-top:16px}.discussion details[open]{margin-bottom:20px}
.agenda-table{max-width:1400px}.agenda-table table{width:100%;border-collapse:collapse;font-size:25px}.agenda-table td{padding:10px 12px;border-bottom:1px solid #d2e0ea}.agenda-table td:first-child{color:#7b263d;width:120px}.agenda-table td:last-child{text-align:right;width:160px}.schedule-note{font-size:22px}
.tex-error{border:2px solid #9b3045;color:#9b3045}.figure-zoom{position:absolute;right:8px;top:7px;z-index:2;font:16px "CMU Serif",serif!important;padding:4px 8px;background:#ffffffee}.stage{position:relative}.figure-modal{width:95vw;max-width:1800px;border:1px solid #c4d9e8;padding:22px;background:#fff}.figure-modal::backdrop{background:#fffE}.figure-modal img{display:block;width:100%;max-height:80vh;object-fit:contain}.figure-modal .row{justify-content:space-between}
.newton-history{max-height:240px}.steps p{font-size:28px!important}.energy-row output{font-size:21px!important}.energy-row{grid-template-columns:30px 1fr 125px}.bar{height:13px}.graph-explain{font-size:22px!important}
@media(min-width:951px) and (max-height:800px){body{font-size:24px!important}h2{font-size:40px!important}h3{font-size:26px!important}.equation{font-size:30px!important}p.lead{font-size:26px!important}.small,.note{font-size:20px!important}.controls{padding:14px!important}.control label,.control output{font-size:21px!important}.table{font-size:22px!important}.table th{font-size:20px!important}.slide{padding-top:20px!important}}
@media(max-width:950px){body{font-size:24px!important}h2{font-size:36px!important}h1{font-size:58px!important}.slide{padding:24px!important}.split{grid-template-columns:1fr!important}.course,.hint,.time-label{display:none}.equation{font-size:28px!important}.controls{grid-template-columns:1fr}.brand{font-size:20px!important}.tools button{font-size:16px!important}.references{grid-template-columns:1fr}.figure-modal{padding:12px}.progress{display:none}}
@media print{html,body{height:auto;overflow:visible}header,footer,.figure-zoom,.figure-modal{display:none!important}.slide{height:auto;overflow:visible;display:block!important;break-after:page}.discussion details>div{display:block}.equation{overflow:visible}body{font-size:18px!important}.equation{font-size:24px!important}h2{font-size:32px!important}}
'''

MATH_TOOLS=r'''
function setTex(element,tex,display=true){if(!element)return;element.dataset.tex=tex;element.classList.add(display?'math-tex':'math-inline');try{katex.render(tex,element,{displayMode:display,throwOnError:true,strict:'ignore',trust:false});element.classList.remove('tex-error');}catch(error){element.classList.add('tex-error');element.textContent=tex;console.error(error);}}
function typesetAll(root=document){root.querySelectorAll('[data-tex]').forEach(el=>setTex(el,el.dataset.tex,!el.classList.contains('math-inline')));}
function currentOriginal(){const s=slides[active];return s&&s.dataset.originalIndex!==undefined?Number(s.dataset.originalIndex):-1;}
function showFigure(canvas){const dialog=document.getElementById('figure-modal');const img=dialog.querySelector('img');img.src=canvas.toDataURL('image/png');img.alt=canvas.getAttribute('aria-label')||'Увеличенный рисунок';dialog.showModal();}
typesetAll();
'''

TAIL=r'''
// Маршрут и графические подписи синхронизируются после загрузки встроенных шрифтов.
const scheduledGo=go;go=function(n){scheduledGo(n);slides[active].scrollTop=0;const slide=slides[active];const minutes=Number(slide.dataset.minutes||2);const elapsed=slides.slice(0,active).reduce((s,e)=>s+Number(e.dataset.minutes||2),0);document.getElementById('time-label').textContent=`${minutes} мин · ${elapsed}–${elapsed+minutes} / 90`;};window.deck.go=go;
document.querySelectorAll('.stage').forEach(stage=>{const canvas=stage.querySelector('canvas');if(!canvas)return;const button=document.createElement('button');button.type='button';button.className='figure-zoom';button.textContent='Увеличить ⤢';button.setAttribute('aria-label','Увеличить рисунок');button.onclick=()=>showFigure(canvas);stage.append(button);canvas.addEventListener('dblclick',()=>showFigure(canvas));});
document.getElementById('figure-close').onclick=()=>document.getElementById('figure-modal').close();
document.addEventListener('keydown',event=>{if(document.getElementById('figure-modal').open){if(event.key==='Escape'){event.preventDefault();document.getElementById('figure-modal').close();}event.stopImmediatePropagation();}},true);
document.fonts.ready.then(()=>{typesetAll();drawAll();document.documentElement.dataset.fontsReady='true';});go(active);
'''


def configure_equation(el,tex):
    # Сохраняем id и hidden; заменяем только содержимое формулы.
    for child in list(el):el.remove(child)
    el.text=None
    el.set('data-tex',tex)
    classes=set(el.get('class','').split());classes.add('math-tex');el.set('class',' '.join(sorted(classes)))


def patch_code(code,lecture):
    code=code.replace('switch(active)','switch(currentOriginal())')
    code=code.replace('if(active===','if(currentOriginal()===')
    code=code.replace('[0,8,9,10].includes(active)','[0,8,9,10].includes(currentOriginal())')
    # После go вызывается drawAll; исходные номера нужны только графическим интерактивам.
    if '// CMU_CANVAS_READY' not in code:
        code=code.replace('Georgia,serif','CMU Serif,serif')
        code=re.sub(r"font='(\d+)px CMU Serif,serif'", lambda m: "font='"+str(min(30,max(16,round(int(m.group(1))*1.33))))+"px CMU Serif,serif'", code)
        code=code.replace("c.font=size+'px CMU Serif,serif'", "c.font=(size*1.18)+'px CMU Serif,serif'")
        code+='\n// CMU_CANVAS_READY\n'
    if lecture=='lecture_3_bond_graphs':
        code=code.replace("c.font=size+'px Georgia,serif'", "c.font=(size*1.18)+'px \\\"CMU Serif\\\",serif'")
        # Динамические формулы не должны снова превращаться в плоский текст.
        code=code.replace("$('elementEquation').textContent=equation;", "setTex($('elementEquation'),element==='C'?'\\\\dot q=f,\\\\quad e=\\\\frac q C,\\\\quad H=\\\\frac{q^2}{2C}':element==='I'?'\\\\dot p=e,\\\\quad f=\\\\frac p I,\\\\quad H=\\\\frac{p^2}{2I}':'e=Rf,\\\\quad P_R=Rf^2\\\\ge0');")
        code=code.replace("$('junctionLaw').textContent=junction===0?'e₁=e₂=e₃; f₁=f₂+f₃':'f₁=f₂=f₃; e₁=e₂+e₃';", "setTex($('junctionLaw'),junction===0?'e_1=e_2=e_3,\\\\quad f_1=f_2+f_3':'f_1=f_2=f_3,\\\\quad e_1=e_2+e_3');")
    if lecture=='lecture_2_2R_robot':
        pattern=r"\$\('massValue'\)\.textContent=`.*?`;"
        replacement="setTex($('massValue'),'M=\\\\begin{bmatrix}'+fmt(m[0][0],4)+'&'+fmt(m[0][1],4)+'\\\\\\\\'+fmt(m[1][0],4)+'&'+fmt(m[1][1],4)+'\\\\end{bmatrix}');"
        code=re.sub(pattern,lambda m:replacement,code,flags=re.S)
        pattern=r"\$\('acceleration'\)\.textContent=`.*?`;"
        replacement="setTex($('acceleration'),'\\\\ddot q=\\\\begin{bmatrix}'+fmt(a[0],2)+'\\\\\\\\'+fmt(a[1],2)+'\\\\end{bmatrix}\\\\,\\\\mathrm{rad/s^2}');"
        code=re.sub(pattern,lambda m:replacement,code,flags=re.S)
    return code


def assign_minutes(sections):
    for i,s in enumerate(sections):
        kind=s.get('data-kind','interactive')
        original=s.get('data-original-index')
        n=3 if kind=='activity' else 2
        if i==0 or i==len(sections)-1:n=1
        s.set('data-minutes',str(n))
    total=sum(int(s.get('data-minutes')) for s in sections)
    candidates=[s for s in sections if s.get('data-kind')=='derivation']+[s for s in sections[1:-1] if s.get('data-kind')!='activity']
    idx=0
    while total!=90:
        s=candidates[idx%len(candidates)];n=int(s.get('data-minutes'))
        if total<90 and n<4:s.set('data-minutes',str(n+1));total+=1
        elif total>90 and n>1:s.set('data-minutes',str(n-1));total-=1
        idx+=1
        if idx>2000:raise RuntimeError('Невозможно назначить время')


def main():
    fonts=font_css();katex_js=(BASE/'katex-0.16.22/dist/katex.min.js').read_text().replace('</script','<\\/script')
    for lecture,expansions in EXPANSIONS.items():
        path=ROOT/lecture/'presentation/index.html'
        doc=html.fromstring(path.read_text())
        for el in doc.xpath('//*[@id="cmu-upgrade" or @id="katex-runtime" or @id="math-tools" or @id="schedule-tools" or @id="figure-modal" or @id="time-label"]'):
            el.getparent().remove(el)
        main=doc.find('.//main')
        originals=[s for s in main.findall('section') if s.get('data-extra')!='true']
        for s in list(main):main.remove(s)
        output=[]
        for i,section in enumerate(originals):
            section.set('data-original-index',str(i))
            section.set('data-kind','interactive')
            equations=section.xpath('.//*[contains(concat(" ",normalize-space(@class)," ")," equation ")]')
            if i in FORMULAS[lecture]:
                values=FORMULAS[lecture][i]
                assert len(values)==len(equations),(lecture,i,len(equations),len(values))
                for node,tex in zip(equations,values):configure_equation(node,tex)
            if lecture=='lecture_1_modelling_n_simulation' and i==4:
                steps=[r'\frac{\partial L}{\partial\dot x}=m\dot x',r'\frac{d}{dt}(m\dot x)=m\ddot x',r'\frac{\partial L}{\partial x}=-kx-mg',r'm\ddot x+kx+mg=u-b\dot x',r'm\ddot x+b\dot x+kx+mg=u']
                for el,tex in zip(section.xpath('.//*[@id="lagrangeSteps"]/p'),steps):configure_equation(el,tex)
            if lecture=='lecture_2_2R_robot' and i==5:
                for el in section.xpath('.//p[contains(@class,"small")]'):
                    if '½' in el.text_content():configure_equation(el,r'T=\frac12\sum_i(m_iv_{ci}^2+I_i\omega_i^2)')
            if lecture=='lecture_1_modelling_n_simulation' and i==6:
                rows=section.xpath('.//table//tr[td="Колебательное затухание"]')
                if rows:
                    rows[0][0].text='0 < ζ < 1'
                    table=rows[0].getparent()
                    if not table.xpath('./tr[td="ζ = 0"]'):
                        table.insert(0,html.fragment_fromstring('<tr><td>ζ = 0</td><td>Незатухающие колебания</td></tr>'))
            output.append(section)
            for item in expansions.get(i,[]):
                new=html.fragment_fromstring('<section class="slide" data-extra="true"></section>')
                new.set('data-title',item['title']);new.set('aria-label',item['title']);new.set('data-kind',item['kind'])
                for node in html.fragments_fromstring(item['body']):new.append(node)
                output.append(new)
        assign_minutes(output)
        for s in output:main.append(s)
        # Программа занятия делится на шесть последовательных блоков, сумма 90.
        agenda_node=doc.xpath('//*[contains(concat(" ",normalize-space(@class)," ")," agenda-table ")]')[0]
        group_size=(len(output)+5)//6
        rows=[];elapsed=0
        for start in range(0,len(output),group_size):
            group=output[start:start+group_size];duration=sum(int(s.get('data-minutes')) for s in group)
            name=group[0].get('data-title')+' → '+group[-1].get('data-title')
            rows.append(f'<tr><td>{elapsed}–{elapsed+duration}</td><td>{name}</td><td>{duration} мин</td></tr>');elapsed+=duration
        agenda_node.append(html.fragment_fromstring('<table>'+''.join(rows)+'</table>'))
        head=doc.find('head');style=etree.SubElement(head,'style',id='cmu-upgrade');style.text=fonts+'\n'+STYLE
        body=doc.find('body')
        deck_script=doc.xpath('//script[@id="deck-code"]')
        if deck_script:deck_script=deck_script[0]
        else:deck_script=[s for s in body.findall('script') if not s.get('id') and "'use strict';" in (s.text or '')][0]
        deck_script.set('id','deck-code');deck_script.text=patch_code(deck_script.text,lecture)
        at=body.index(deck_script)
        runtime=etree.Element('script',id='katex-runtime');runtime.text=katex_js;body.insert(at,runtime)
        tools=etree.Element('script',id='math-tools');tools.text=MATH_TOOLS;body.insert(at+1,tools)
        tail=etree.Element('script',id='schedule-tools');tail.text=TAIL;body.insert(body.index(deck_script)+1,tail)
        footer=doc.find('.//footer');time=etree.Element('span',id='time-label',attrib={'class':'time-label'});footer.insert(1,time)
        dialog=html.fragment_fromstring('<dialog id="figure-modal" class="figure-modal"><div class="row"><h3>Увеличенный рисунок · текущий кадр</h3><button id="figure-close">Закрыть ×</button></div><img alt="Увеличенный рисунок"></dialog>');body.insert(body.index(deck_script),dialog)
        path.write_text('<!doctype html>\n'+html.tostring(doc,encoding='unicode',method='html'))
        plan=['# План лекции — 90 минут','',f'Всего {len(output)} слайдов. CMU Serif встроен; формулы набраны KaTeX из LaTeX. Время включает обсуждение и практические паузы.','','| № | Слайд | Минут | Интервал |','|---:|---|---:|---|']
        elapsed=0
        for n,s in enumerate(output,1):
            m=int(s.get('data-minutes'));plan.append(f'| {n} | {s.get("data-title")} | {m} | {elapsed}–{elapsed+m} |');elapsed+=m
        plan+=['','**Итого: 90 минут.** Это рекомендуемый маршрут, а не обещание одинакового темпа для любой аудитории.','', 'Формулы и шрифты встроены в HTML; подключение к сети не требуется. Плотные страницы можно прокручивать, рисунки — увеличивать кнопкой ⤢.']
        (path.parent/'TEACHING_PLAN.md').write_text('\n'.join(plan)+'\n')
        print(lecture, len(output),'слайдов;',elapsed,'минут;',round(path.stat().st_size/1024),'KiB')

if __name__=='__main__':main()
