// HJ_261118 - 손님 화면 한·영 전환.
// 기존 화면 코드는 한국어 문장을 그대로 쓰고, 표시 직전에 t(문장)으로 감싼다.
// 영어가 선택되면 아래 사전(정확히 일치) → 패턴(숫자가 섞인 문장) 순서로 찾고,
// 둘 다 없으면 한국어를 그대로 보여 준다(번역 누락이 화면을 깨뜨리지 않게).
// 관리자 화면은 번역하지 않는다.
import { useCallback, useSyncExternalStore } from 'react';

export const LANGS = ['ko', 'en'];
const DEFAULT_LANG = 'ko';

// ── 아주 작은 전역 저장소: Provider 없이 어느 컴포넌트에서든 같은 언어를 본다 ──
let currentLang = DEFAULT_LANG;
const listeners = new Set();

export function getLang() {
  return currentLang;
}

export function setLang(lang) {
  const next = LANGS.includes(lang) ? lang : DEFAULT_LANG;
  if (next === currentLang) return;
  currentLang = next;
  try { document.documentElement.lang = next; } catch (_) { /* no DOM in some tests */ }
  listeners.forEach(listener => listener());
}

export function resetLang() {
  setLang(DEFAULT_LANG);
}

function subscribe(listener) {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

export function useLang() {
  return useSyncExternalStore(subscribe, getLang, getLang);
}

import { EN_DRIVING } from './i18n.en.driving';
import { EN_GUEST } from './i18n.en.guest';

// ── 사전: 한국어 원문 → 영어 ──
// 주행·충전 화면 문구는 i18n.en.driving.js 에 따로 둔다.
export const EN = {
  ...EN_DRIVING,
  ...EN_GUEST,
  // 상단 배너
  '국립공원공단 · Woraksan National Park': 'Korea National Park Service · Woraksan National Park',
  // 배너 한 줄에 들어가도록 짧게(공원 이름은 윗줄에 이미 있다).
  '월악산 국립공원 배송 로봇': 'Woraksan Delivery Robot',
  '언어 선택': 'Language',

  // 운행 상태 카드
  '로봇 운행 상태': 'Robot status',
  '시스템': 'System',
  '임무': 'Mission',
  '주차': 'Parking',
  '배터리': 'Battery',
  '시스템 시작 중': 'Starting up',
  '시스템 정상': 'All systems normal',
  '시스템 경고': 'System warning',
  '시스템 오류': 'System error',
  '시스템 준비 중': 'Getting ready',
  '주행 준비 완료': 'Ready to drive',
  '목적지 접수': 'Destination received',
  '경로 준비 중': 'Planning route',
  '주행 중': 'Driving',
  '안전 정지': 'Safety stop',
  '도착': 'Arrived',
  '운행 정지': 'Stopped',
  '충전 완료': 'Fully charged',
  '충전 중': 'Charging',
  '충전 접점 연결 대기 중': 'Waiting for charger contact',
  '대기·충전 장소 주차 완료': 'Parked at the station',
  '대기·충전 장소 주차 중': 'Parking at the station',
  '충전 도킹 중': 'Docking to charge',
  '후진 주차 중': 'Reverse parking',
  '충전 도킹 선택됨': 'Charge docking selected',
  '일반 후진 주차 선택됨': 'Reverse parking selected',
  '충전 도킹 필요': 'Charging needed',
  '자동 주차 · 배터리 35% 기준': 'Auto parking · 35% battery rule',
  '배터리 상태 확인 중': 'Checking battery',

  // 실증 운행 현황
  '실증 운행 현황': 'Service record',
  '실증 운행 현황 상세 보기': 'Open service record details',
  '상세 보기 ›': 'Details ›',
  '이번 서비스': 'This trip',
  '최근 서비스': 'Last trip',
  '오늘 누적': 'Today',
  '오늘 완료': 'Done today',
  '전체 누적': 'All time',
  '불러오는 중': 'Loading',
  '확인 불가': 'Unavailable',
  '기록 없음': 'No records',
  '최근 집계 표시 중 · 갱신 지연': 'Showing last totals · update delayed',

  // 서비스 선택 카드와 안내 카드
  '지금 바로 출발할 수 있어요': 'Ready to go now',
  '현재는 운영시간이 아닙니다': 'Currently outside operating hours',
  '현재는 운영시간이 아닙니다.': 'We are currently closed.',
  '서비스 선택': 'Select a Service',
  '캠핑 사이트까지 무거운 짐을': 'Let the robot carry your heavy bags',
  '로봇이 대신 옮겨드려요.': 'to your campsite.',
  '시작하기': 'Start',
  '로봇 이용 방법': 'How to Use',
  '시설 안내': 'Facilities',
  '탐방로': 'Hiking Trails',
  '짐 싣기부터 받기까지 3단계': 'Load, send and pick up in 3 steps',
  '화장실 · 개수대 · 매점 · 관리사무소': 'Restrooms · Sinks · Store · Office',
  '코스별 거리와 난이도': 'Distance and difficulty by course',
  '확인': 'OK',
  '뒤로가기': 'Back',
};

// ── 숫자가 섞인 문장: 전체가 일치할 때만 바꾼다 ──
const EN_PATTERNS = [
  [/^임무 가능 · 배터리 (\d+)%$/, 'Ready · Battery $1%'],
  [/^(\d+)% 미만 · 새 임무 대기 \((\d+)%\)$/, 'Below $1% · New trips paused ($2%)'],
  [/^25% 미만 · 긴급 복귀 필요 \((\d+)%\)$/, 'Below 25% · Returning to charge ($1%)'],
  [/^배터리 부족 · 즉시 복귀\s*(\d+%)?$/, 'Low battery · Returning now $1'],
  [/^배터리 부족 복귀 중\s*(\d+%)?$/, 'Low battery · Returning $1'],
  [/^이용 완료 후 복귀 대기\s*(\d+%)?$/, 'Will return after you finish $1'],
  [/^임무 완료 후 복귀\s*(\d+%)?$/, 'Will return after this trip $1'],
  [/^(.+) 기준$/, 'As of $1'],
  [/^시연 크기 (.+ m)$/, 'Demo size $1'],
  [/^([A-Z]\d{1,2}) 출차에 실패했습니다\. 같은 사이트를 다시 선택해 주세요\.$/, 'Could not leave for $1. Please select the same site again.'],
  [/^배터리 (\d+(?:\.\d+)?)% 이상과 정상 차량 상태가 필요합니다\.$/, 'Needs at least $1% battery and a healthy robot.'],
  [/^충전 위치를 최신 지도로 확인할 수 없습니다\. \((.*)\)$/, 'Cannot confirm the charger position on the map. ($1)'],
  [/^관측 범위 (.+ m)$/, 'Observed $1'],
  // 사이트·구역 (손님 화면)
  [/^([A-Z]\d{1,2}) 사이트에 도착했어요!$/, 'Arrived at site $1!'],
  [/^([A-Z]\d{1,2}) 사이트 앞에 도착했어요!$/, 'Arrived in front of site $1!'],
  [/^([A-Z]\d{1,2}) 앞 도로로 부를까요\?$/, 'Call the robot to the road in front of $1?'],
  [/^([A-Z]\d{1,2})로 배달할까요\?$/, 'Deliver to $1?'],
  [/^([A-Z]\d{1,2}) 호출 사이트$/, 'Pickup site $1'],
  [/^([A-Z]\d{1,2}) 호출$/, 'Pickup · $1'],
  [/^예: ([A-Z]\d{1,2})$/, 'e.g. $1'],
  [/^([A-Z])구역$/, 'Zone $1'],
  [/^([A-Z]) 구역$/, 'Zone $1'],
  [/^([A-Z])구역 배치도$/, 'Zone $1 map'],
  [/^([A-Z])구역에서 로봇을 부를 사이트를 골라주세요$/, 'Zone $1 · Choose where to call the robot'],
  [/^([A-Z])구역에서 짐을 받을 사이트를 골라주세요$/, 'Zone $1 · Choose the site for your bags'],
  [/^텐트 (\d+)곳$/, '$1 with tents'],
  [/^빈 자리 (\d+)곳$/, '$1 free'],
  [/^현재 배터리 (\d+)% · 배터리를 충전하고 있습니다\.$/, 'Battery $1% · Charging.'],
  // 배터리 안내 팝업
  [/^배터리 잔량이 (\d+)%입니다\. 새 사이트 이동은 (\d+)% 이상에서만 가능합니다\.$/, 'Battery is at $1%. New trips need at least $2%.'],
  [/^배터리 상태를 아직 받지 못했습니다\. 새 사이트 이동은 (\d+)% 이상 확인 후 가능합니다\.$/, 'Battery level not received yet. New trips need at least $1%.'],
  [/^(?:배터리 잔량이 (\d+)%입니다\.|배터리 잔량이 낮습니다\.) 25% 미만으로 현재 작업을 중단하고 충전 도킹을 위해 즉시 복귀합니다\. 로봇 주변을 비워주세요\.$/, 'Battery low ($1%). Below 25%, the robot stops this task and returns to charge now. Please clear the area.'],
  [/^(?:배터리 잔량이 (\d+)%입니다\.|배터리 잔량이 낮습니다\.) 복귀 요청이 확인되어 충전 구역으로 이동합니다\.$/, 'Battery low ($1%). Return confirmed; heading to the charging area.'],
  [/^(?:배터리 잔량이 (\d+)%입니다\.|배터리 잔량이 낮습니다\.) 짐 처리를 마친 뒤 이용 완료 버튼을 눌러주세요\. 완료 후 충전 도킹을 위해 복귀합니다\.$/, 'Battery low ($1%). Tap Finish when your bags are done; the robot will then return to charge.'],
  [/^(?:배터리 잔량이 (\d+)%입니다\.|배터리 잔량이 낮습니다\.) (\d+)% 미만이면 새 임무를 받지 않고, 현재 임무 완료 후 충전 구역 복귀를 대기합니다\.$/, 'Battery low ($1%). Below $2% no new trips are taken; the robot returns to charge after this one.'],
  [/^([\d,]+)회$/, '$1 trips'],
  [/^(.+) · ([\d,]+)회$/, '$1 · $2 trips'],
];

export function translate(lang, text) {
  if (lang !== 'en' || typeof text !== 'string' || !text) return text;
  if (Object.prototype.hasOwnProperty.call(EN, text)) return EN[text];
  for (const [pattern, replacement] of EN_PATTERNS) {
    // 선택 그룹이 비면 '(%)' 같은 빈 괄호가 남으므로 지운다.
    if (pattern.test(text)) return text.replace(pattern, replacement).replace(/ \(%\)/g, '').replace(/\s+$/, '');
  }
  return text;
}

export function useT() {
  const lang = useLang();
  return useCallback(text => translate(lang, text), [lang]);
}

// 시계·날짜 표기도 언어를 따른다.
export function localeOf(lang) {
  return lang === 'en' ? 'en-US' : 'ko-KR';
}

// 모듈 수준에 미리 만들어 둔 JSX(안내 팝업 내용 등)에서도 쓸 수 있는 번역 컴포넌트.
// 렌더링 시점의 언어로 바뀐다: <T>시설 안내</T>
export function T({ children }) {
  const t = useT();
  return t(children);
}
