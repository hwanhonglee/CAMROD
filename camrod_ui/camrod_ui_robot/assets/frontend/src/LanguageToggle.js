import React from 'react';
import { setLang, useLang } from './i18n';

// HJ_261118 - 배너의 한·영 전환. 각 언어 이름을 그 언어로 써서 한글을 몰라도 찾을 수 있게 한다.
export default function LanguageToggle() {
  const lang = useLang();
  return (
    <div className="wh-lang-toggle" role="group" aria-label={lang === 'en' ? 'Language' : '언어 선택'}>
      {[['ko', '한국어'], ['en', 'English']].map(([code, label]) => (
        <button
          key={code}
          type="button"
          lang={code}
          data-ui={`operator-lang-${code}`}
          className={lang === code ? 'active' : ''}
          aria-pressed={lang === code}
          onClick={e => { e.stopPropagation(); setLang(code); }}
        >
          {label}
        </button>
      ))}
    </div>
  );
}
