import { EN, getLang, resetLang, setLang, translate } from './i18n';

// HJ_261118 - 손님 화면 한·영 전환: 사전, 숫자 문장 패턴, 누락 시 한국어 유지.
afterEach(() => resetLang());

test('Korean is the default and unknown languages fall back to Korean', () => {
  expect(getLang()).toBe('ko');
  setLang('en');
  expect(getLang()).toBe('en');
  setLang('fr');
  expect(getLang()).toBe('ko');
});

test('Korean mode never rewrites text', () => {
  expect(translate('ko', '시작하기')).toBe('시작하기');
});

test('English uses the dictionary, then number patterns, else keeps Korean', () => {
  expect(translate('en', '시작하기')).toBe('Start');
  expect(translate('en', '임무 가능 · 배터리 80%')).toBe('Ready · Battery 80%');
  expect(translate('en', '35% 미만 · 새 임무 대기 (30%)')).toBe('Below 35% · New trips paused (30%)');
  expect(translate('en', '12.3 km · 4회')).toBe('12.3 km · 4 trips');
  expect(translate('en', '사전에 없는 문장')).toBe('사전에 없는 문장');
});

test('every dictionary entry has a non-empty English value without Hangul', () => {
  Object.entries(EN).forEach(([ko, en]) => {
    expect(typeof en).toBe('string');
    expect(en.trim()).not.toBe('');
    expect(/[가-힣]/.test(en)).toBe(false);
  });
});

test('guest-flow sentences with site names and numbers translate', () => {
  expect(translate('en', 'B7 사이트에 도착했어요!')).toBe('Arrived at site B7!');
  expect(translate('en', 'B10로 배달할까요?')).toBe('Deliver to B10?');
  expect(translate('en', 'C구역에서 짐을 받을 사이트를 골라주세요')).toBe('Zone C · Choose the site for your bags');
  expect(translate('en', '빈 자리 5곳')).toBe('5 free');
  expect(translate('en', '배터리 잔량이 낮습니다. 복귀 요청이 확인되어 충전 구역으로 이동합니다.'))
    .toBe('Battery low. Return confirmed; heading to the charging area.');
});
