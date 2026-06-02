import React from 'react';

export default function AMRAnimation() {

  /* ── Far mountains ── */
  const farMtns = (ox) => (
    <path
      d={`M${ox},222 Q${ox+80},138 ${ox+165},188 Q${ox+245},132 ${ox+328},178
          Q${ox+410},115 ${ox+490},160 Q${ox+572},120 ${ox+655},168
          Q${ox+730},133 ${ox+800},222 L${ox+800},252 L${ox},252 Z`}
      fill="#8aaac0" opacity="0.62"
    />
  );

  /* ── Mid mountains + pines + tents ── */
  const midMtns = (ox) => (
    <>
      <path
        d={`M${ox},268 Q${ox+62},194 ${ox+145},236 Q${ox+205},176 ${ox+285},222
            Q${ox+365},172 ${ox+445},216 Q${ox+525},184 ${ox+608},226
            Q${ox+685},190 ${ox+762},230 Q${ox+783},238 ${ox+800},268
            L${ox+800},312 L${ox},312 Z`}
        fill="#4e7038" opacity="0.88"
      />
      {[35,118,205,328,448,568,688,775].map((x, i) => (
        <g key={i} transform={`translate(${ox+x},242)`}>
          <rect x="-4" y="28" width="8" height="22" fill="#5a3010" />
          <polygon points="0,-8 -14,22 14,22"  fill="#1e4a12" />
          <polygon points="0,-22 -11,6 11,6"   fill="#285e1a" />
          <polygon points="0,-36 -9,-8 9,-8"   fill="#326e22" />
        </g>
      ))}
      {[168,388,598].map((x, i) => (
        <g key={i} transform={`translate(${ox+x},272)`}>
          <polygon points="0,-18 -16,6 16,6"  fill="#ede5cc" />
          <polygon points="0,-18 2,-2 -2,-2"  fill="#c8c0a0" />
          <rect x="-4" y="0" width="8" height="6" fill="#22201a" />
        </g>
      ))}
    </>
  );

  /* ── Near autumn trees + leaves ── */
  const nearTrees = (ox) => (
    <>
      <g transform={`translate(${ox+42},265)`}>
        <rect x="-6" y="0" width="12" height="45" fill="#6b3a12" />
        <ellipse cx="0"   cy="-20" rx="38" ry="32" fill="#c84020" opacity="0.92" />
        <ellipse cx="22"  cy="-8"  rx="24" ry="20" fill="#e05828" opacity="0.78" />
        <ellipse cx="-20" cy="-6"  rx="20" ry="18" fill="#d03010" opacity="0.85" />
      </g>
      <g transform={`translate(${ox+168},268)`}>
        <rect x="-5" y="0" width="10" height="42" fill="#6b4a15" />
        <ellipse cx="0"  cy="-24" rx="32" ry="28" fill="#e8b820" opacity="0.92" />
        <ellipse cx="16" cy="-12" rx="20" ry="18" fill="#f0c825" opacity="0.78" />
      </g>
      <g transform={`translate(${ox+278},260)`}>
        <rect x="-5" y="32" width="10" height="38" fill="#5a3010" />
        <polygon points="0,-14 -22,30 22,30"  fill="#1a4015" />
        <polygon points="0,-30 -18,8 18,8"    fill="#22501c" />
        <polygon points="0,-46 -14,-8 14,-8"  fill="#2a6024" />
      </g>
      <g transform={`translate(${ox+408},263)`}>
        <rect x="-6" y="0" width="12" height="45" fill="#6b3a12" />
        <ellipse cx="0"   cy="-22" rx="36" ry="30" fill="#e06030" opacity="0.92" />
        <ellipse cx="-22" cy="-9"  rx="24" ry="20" fill="#c84020" opacity="0.78" />
      </g>
      <g transform={`translate(${ox+535},266)`}>
        <rect x="-5" y="0" width="10" height="42" fill="#6b4a15" />
        <ellipse cx="0"   cy="-22" rx="34" ry="26" fill="#d8a815" opacity="0.92" />
        <ellipse cx="-16" cy="-10" rx="22" ry="18" fill="#f0c020" opacity="0.78" />
      </g>
      <g transform={`translate(${ox+665},258)`}>
        <rect x="-5" y="32" width="10" height="40" fill="#5a3010" />
        <polygon points="0,-16 -24,30 24,30"  fill="#1a4015" />
        <polygon points="0,-32 -20,8 20,8"    fill="#22501c" />
        <polygon points="0,-48 -16,-8 16,-8"  fill="#2a6024" />
      </g>
      <g transform={`translate(${ox+775},265)`}>
        <rect x="-6" y="0" width="12" height="45" fill="#6b3a12" />
        <ellipse cx="0" cy="-20" rx="35" ry="30" fill="#d03018" opacity="0.92" />
      </g>
      {[52,95,152,208,295,358,458,528,605,682,748].map((x, i) => (
        <ellipse key={i} cx={ox+x} cy={309+(i%3)*4} rx={6+(i%3)} ry="3"
          fill={['#c84020','#e8a020','#d06030'][i%3]} opacity="0.82"
          transform={`rotate(${i*27},${ox+x},${309+(i%3)*4})`} />
      ))}
    </>
  );

  /* ── Wheel: static tire + rotating hub ── */
  const Wheel = ({ cx, cy }) => {
    const spokes = [0, 60, 120, 180, 240, 300].map(deg => {
      const r = (deg * Math.PI) / 180;
      return {
        x1: cx + Math.sin(r) * 7,  y1: cy - Math.cos(r) * 7,
        x2: cx + Math.sin(r) * 22, y2: cy - Math.cos(r) * 22,
      };
    });
    return (
      <>
        {/* ── 정적 타이어 ── */}
        <circle cx={cx} cy={cy} r="31" fill="#141414" />
        {/* 타이어 트레드 노치 */}
        {[0,22.5,45,67.5,90,112.5,135,157.5,180,202.5,225,247.5,270,292.5,315,337.5].map((deg, i) => {
          const r = (deg * Math.PI) / 180;
          const mx = cx + Math.sin(r) * 28;
          const my = cy - Math.cos(r) * 28;
          return <circle key={i} cx={mx} cy={my} r="2.2" fill="#2a2a2a" />;
        })}
        <circle cx={cx} cy={cy} r="24" fill="#222" />

        {/* ── 회전 허브 ── */}
        <g>
          <animateTransform attributeName="transform" type="rotate"
            from={`0 ${cx} ${cy}`} to={`360 ${cx} ${cy}`}
            dur="1.4s" repeatCount="indefinite" />
          {/* 6 스포크 */}
          {spokes.map((s, i) => (
            <line key={i} x1={s.x1} y1={s.y1} x2={s.x2} y2={s.y2}
              stroke="#585858" strokeWidth="3.5" strokeLinecap="round" />
          ))}
          {/* 허브 링 */}
          <circle cx={cx} cy={cy} r="9"  fill="#6a6a6a" />
          <circle cx={cx} cy={cy} r="5"  fill="#a0a0a0" />
          <circle cx={cx} cy={cy} r="2"  fill="#d0d0d0" />
        </g>

        {/* 타이어 하이라이트 */}
        <path d={`M${cx-13},${cy-26} Q${cx},${cy-32} ${cx+13},${cy-26}`}
          fill="none" stroke="rgba(255,255,255,0.12)" strokeWidth="3.5" strokeLinecap="round" />
      </>
    );
  };

  return (
    <div style={{ width:'100%', height:'100%', overflow:'hidden', borderRadius:'16px' }}>
      <svg viewBox="0 0 800 380" preserveAspectRatio="xMidYMid meet"
        xmlns="http://www.w3.org/2000/svg"
        overflow="hidden"
        style={{ width:'100%', height:'100%', display:'block' }}>

        <defs>
          {/* 하늘 */}
          <linearGradient id="amr-sky" x1="0" y1="0" x2="0" y2="1" gradientUnits="objectBoundingBox">
            <stop offset="0%"   stopColor="#1560a5" />
            <stop offset="48%"  stopColor="#4aa4d8" />
            <stop offset="100%" stopColor="#b8e2f5" />
          </linearGradient>
          {/* 도로 */}
          <linearGradient id="amr-road" x1="0" y1="0" x2="0" y2="1" gradientUnits="objectBoundingBox">
            <stop offset="0%"   stopColor="#a08060" />
            <stop offset="100%" stopColor="#7a5830" />
          </linearGradient>
          {/* 로봇 바디 */}
          <linearGradient id="rb-body" x1="278" y1="238" x2="310" y2="322" gradientUnits="userSpaceOnUse">
            <stop offset="0%"   stopColor="#ddeef8" />
            <stop offset="55%"  stopColor="#b0ccde" />
            <stop offset="100%" stopColor="#8aaec8" />
          </linearGradient>
          {/* 바디 상단 패널 */}
          <linearGradient id="rb-upper" x1="0" y1="0" x2="0" y2="1" gradientUnits="objectBoundingBox">
            <stop offset="0%"   stopColor="#e8f4fc" />
            <stop offset="100%" stopColor="#c2d8ea" />
          </linearGradient>
          {/* 센서 하우징 */}
          <linearGradient id="rb-sensor" x1="0" y1="0" x2="0" y2="1" gradientUnits="objectBoundingBox">
            <stop offset="0%"   stopColor="#243444" />
            <stop offset="100%" stopColor="#131f2c" />
          </linearGradient>
          {/* 디스플레이 */}
          <linearGradient id="rb-display" x1="0" y1="0" x2="0" y2="1" gradientUnits="objectBoundingBox">
            <stop offset="0%"   stopColor="#020d18" />
            <stop offset="100%" stopColor="#061828" />
          </linearGradient>
          {/* 카메라 포드 */}
          <linearGradient id="rb-cam" x1="0" y1="0" x2="1" y2="0" gradientUnits="objectBoundingBox">
            <stop offset="0%"   stopColor="#1a2d3e" />
            <stop offset="100%" stopColor="#0d1c2a" />
          </linearGradient>
          {/* 언더바디 */}
          <linearGradient id="rb-under" x1="0" y1="0" x2="0" y2="1" gradientUnits="objectBoundingBox">
            <stop offset="0%"   stopColor="#2e3e50" />
            <stop offset="100%" stopColor="#1c2c3c" />
          </linearGradient>
          {/* 그림자 */}
          <radialGradient id="rb-shadow" cx="50%" cy="50%" r="50%">
            <stop offset="0%"   stopColor="#000" stopOpacity="0.25" />
            <stop offset="100%" stopColor="#000" stopOpacity="0" />
          </radialGradient>
          {/* 센서 글로우 필터 */}
          <filter id="glow-cyan" x="-20%" y="-60%" width="140%" height="220%">
            <feGaussianBlur in="SourceGraphic" stdDeviation="2.5" result="blur" />
            <feComposite in="SourceGraphic" in2="blur" operator="over" />
          </filter>
          {/* 카메라 글로우 */}
          <filter id="glow-blue" x="-40%" y="-40%" width="180%" height="180%">
            <feGaussianBlur in="SourceGraphic" stdDeviation="2" result="blur" />
            <feComposite in="SourceGraphic" in2="blur" operator="over" />
          </filter>

          <style>{`
            @keyframes amr-scanA {
              0%,100% { opacity: 0.20 }
              50%      { opacity: 0.95 }
            }
            @keyframes amr-scanB {
              0%,100% { opacity: 0.55 }
              50%      { opacity: 0.15 }
            }
            @keyframes amr-pulse {
              0%,100% { opacity: 0.6; r: 4 }
              50%      { opacity: 1.0; r: 5.5 }
            }
            @keyframes amr-wave {
              0%   { transform: translateX(0) }
              100% { transform: translateX(-20px) }
            }
            .amr-sA  { animation: amr-scanA 1.4s ease-in-out infinite }
            .amr-sB  { animation: amr-scanB 1.4s ease-in-out 0.7s infinite }
            .amr-dot { animation: amr-pulse 1.8s ease-in-out infinite }
          `}</style>
        </defs>

        {/* ── Sky ── */}
        <rect x="-1600" width="4000" height="380" fill="url(#amr-sky)" />

        {/* ── Far mountains (30s) ── */}
        <g>
          <animateTransform attributeName="transform" type="translate"
            from="0,0" to="-800,0" dur="30s" repeatCount="indefinite" />
          {farMtns(0)}{farMtns(800)}
        </g>

        {/* ── Mid layer (19s) ── */}
        <g>
          <animateTransform attributeName="transform" type="translate"
            from="0,0" to="-800,0" dur="19s" repeatCount="indefinite" />
          {midMtns(0)}{midMtns(800)}
        </g>

        {/* ── Near trees (11s) ── */}
        <g>
          <animateTransform attributeName="transform" type="translate"
            from="0,0" to="-800,0" dur="11s" repeatCount="indefinite" />
          {nearTrees(0)}{nearTrees(800)}
        </g>

        {/* ── Road ── */}
        <rect x="-1600" y="308" width="4000" height="72" fill="#6a4820" />
        <rect x="-1600" y="310" width="4000" height="48" fill="url(#amr-road)" />
        {[18,52,98,145,192,238,285,335,382,430,480,535,592,645,700,758].map((x, i) => (
          <ellipse key={i} cx={x} cy={318+(i%4)*7} rx={3+(i%3)} ry="2" fill="#6a5030" opacity="0.45" />
        ))}

        {/* ════════════════════════════════
            AMR 로봇  (center ≈ x 383)
            ════════════════════════════════ */}

        {/* 지면 그림자 */}
        <ellipse cx="388" cy="358" rx="115" ry="12" fill="url(#rb-shadow)" />

        {/* ── 타이어 (바디 아래 렌더) ── */}
        <Wheel cx={307} cy={342} />
        <Wheel cx={468} cy={342} />

        {/* ── 언더바디 / 서스펜션 바 ── */}
        <rect x="290" y="318" width="186" height="26" rx="6" fill="url(#rb-under)" />
        {/* 서스펜션 디테일 */}
        <rect x="300" y="322" width="18" height="12" rx="3" fill="#1a2a38" />
        <rect x="458" y="322" width="18" height="12" rx="3" fill="#1a2a38" />
        <line x1="318" y1="328" x2="458" y2="328" stroke="#243444" strokeWidth="2" />

        {/* ── 메인 바디 ── */}
        {/* 바디 그림자/테두리 */}
        <rect x="276" y="236" width="208" height="86" rx="14" fill="#6090b0" opacity="0.35" />
        {/* 메인 바디 */}
        <rect x="278" y="234" width="206" height="86" rx="13" fill="url(#rb-body)" />
        {/* 바디 상단 패널 (밝은 부분) */}
        <rect x="278" y="234" width="206" height="38" rx="13" fill="url(#rb-upper)" />
        <rect x="278" y="258" width="206" height="14" fill="url(#rb-upper)" />
        {/* 청록 액센트 스트라이프 */}
        <rect x="278" y="270" width="206" height="13" rx="0" fill="#0097a7" opacity="0.85" />
        {/* 스트라이프 상하 테두리 라인 */}
        <line x1="278" y1="270" x2="484" y2="270" stroke="#00bcd4" strokeWidth="1" opacity="0.6" />
        <line x1="278" y1="283" x2="484" y2="283" stroke="#006070" strokeWidth="1" opacity="0.6" />

        {/* 바디 테두리 (윤곽선) */}
        <rect x="278" y="234" width="206" height="86" rx="13"
          fill="none" stroke="#7aaabf" strokeWidth="1.5" opacity="0.6" />
        {/* 바디 상단 하이라이트 */}
        <path d="M292,236 Q385,232 468,236"
          fill="none" stroke="rgba(255,255,255,0.55)" strokeWidth="2" strokeLinecap="round" />

        {/* ── 디스플레이 스크린 (빈 화면) ── */}
        <rect x="296" y="245" width="108" height="54" rx="6" fill="url(#rb-display)" />
        <rect x="296" y="245" width="108" height="54" rx="6"
          fill="none" stroke="#1e4060" strokeWidth="1.2" />

        {/* ── 후면 범퍼 ── */}
        <rect x="262" y="247" width="18" height="60" rx="6" fill="url(#rb-cam)" />
        <rect x="262" y="247" width="18" height="60" rx="6"
          fill="none" stroke="#2a3e52" strokeWidth="1" />

        {/* ── 센서 하우징 (상단) ── */}
        {/* 하우징 그림자 */}
        <rect x="293" y="213" width="174" height="26" rx="7" fill="#0a1828" opacity="0.4" />
        {/* 메인 하우징 */}
        <rect x="294" y="211" width="172" height="26" rx="7" fill="url(#rb-sensor)" />
        <rect x="294" y="211" width="172" height="26" rx="7"
          fill="none" stroke="#2a4055" strokeWidth="1.2" />
        {/* 하우징 상단 하이라이트 */}
        <path d="M308,213 Q380,210 450,213"
          fill="none" stroke="rgba(255,255,255,0.18)" strokeWidth="1.5" strokeLinecap="round" />

        {/* 센서 슬릿 (어두운 홈만) */}
        <line x1="308" y1="219" x2="452" y2="219" stroke="#060e18" strokeWidth="3.5" strokeLinecap="round" />
        <line x1="308" y1="225" x2="452" y2="225" stroke="#060e18" strokeWidth="3.5" strokeLinecap="round" />
        <line x1="308" y1="231" x2="452" y2="231" stroke="#060e18" strokeWidth="2.5" strokeLinecap="round" />

        {/* ── 휠 아치 (바디 하단 곡선) ── */}
        <path d="M278,316 Q278,320 284,320 L484,320 Q490,320 490,316"
          fill="#6090b0" opacity="0.25" />

        {/* 먼지 효과 */}
        <ellipse cx="284" cy="354" rx="22" ry="7" fill="#c09050" opacity="0.15" />
        <ellipse cx="490" cy="354" rx="18" ry="6" fill="#c09050" opacity="0.12" />

      </svg>
    </div>
  );
}
