import React from 'react';

// HH_261026 - Categories organize existing view IDs without owning data or commands.
export const ADMIN_DIAGNOSTIC_CATEGORIES = [
  { id: 'driving', label: '주행·안전', tabIds: ['safety', 'trajectory'] },
  { id: 'sensors', label: '센서', tabIds: ['gnss', 'proximity', 'camera'] },
  { id: 'perception', label: '지도·인지', tabIds: ['perception'] },
  { id: 'parking', label: '주차·도킹', tabIds: ['docking'] },
  { id: 'system', label: '시스템', tabIds: ['system', 'snapshot'] },
];

const ADMIN_DIAGNOSTIC_VIEWS = {
  safety: {
    title: '안전 · 제어',
    help: '주행 허가와 안전 정지 원인을 확인합니다.',
  },
  trajectory: {
    title: '주행 궤적',
    help: '위치와 경로, 실제 이동 궤적 및 추종 상태를 확인합니다.',
  },
  gnss: {
    title: 'GNSS · IMU',
    help: '위성 위치와 자세 센서의 수신 상태를 확인합니다.',
  },
  proximity: {
    title: '레이더 · LiDAR',
    help: '주변 거리 센서와 장애물 감지 데이터를 확인합니다.',
  },
  camera: {
    title: '카메라',
    help: '전방·후방·도킹 카메라의 영상과 연결 상태를 확인합니다.',
  },
  perception: {
    title: '지도 · 인지',
    help: '지도와 인지 결과, 장애물 및 비용 지도를 함께 확인합니다.',
  },
  docking: {
    title: '충전 · 주차',
    help: '주차 정책, 충전 접점과 도킹 진행 상태를 확인합니다.',
  },
  system: {
    title: '시스템 상태',
    help: '운영 제어와 장치별 진단을 구분해서 확인합니다.',
  },
  snapshot: {
    title: '스냅샷',
    help: '문제 분석에 필요한 진단 데이터를 확인하고 저장합니다.',
  },
};

export function getAdminDiagnosticView(tabId) {
  const category = ADMIN_DIAGNOSTIC_CATEGORIES.find(group => group.tabIds.includes(tabId));
  return {
    ...(ADMIN_DIAGNOSTIC_VIEWS[tabId] || ADMIN_DIAGNOSTIC_VIEWS.system),
    category: category?.label || '시스템',
  };
}

export default function AdminNavigation({ tabs, activeTab, renderTab }) {
  // HH_261026 - Arrow-key navigation selects only an existing diagnostic view.
  const handleNavigationKey = event => {
    const directions = { ArrowDown: 1, ArrowRight: 1, ArrowUp: -1, ArrowLeft: -1 };
    if (!(event.key in directions) && !['Home', 'End'].includes(event.key)) return;
    if (event.altKey || event.ctrlKey || event.metaKey) return;
    const buttons = Array.from(event.currentTarget.querySelectorAll('[role="tab"]'));
    const index = buttons.indexOf(event.target);
    if (index < 0) return;
    const nextIndex = event.key === 'Home' ? 0 : event.key === 'End' ? buttons.length - 1
      : (index + directions[event.key] + buttons.length) % buttons.length;
    event.preventDefault();
    buttons[nextIndex].focus();
    buttons[nextIndex].click();
  };

  return (
    <nav className="admin-diagnostic-navigation" aria-label="Operator diagnostics views"
      onKeyDown={handleNavigationKey}>
      <div className="admin-navigation-heading">
        <strong>관리자 진단</strong>
        <span>분류별로 화면을 선택하세요</span>
      </div>
      <div className="admin-navigation-groups">
        {ADMIN_DIAGNOSTIC_CATEGORIES.map(category => (
          <section className={`admin-navigation-group ${category.tabIds.includes(activeTab) ? 'is-current' : ''}`}
            key={category.id} aria-labelledby={`admin-category-${category.id}`}>
            <h3 id={`admin-category-${category.id}`}>{category.label}</h3>
            <div className="admin-navigation-tabs" role="tablist" aria-label={category.label}>
              {category.tabIds.map(id => tabs.find(tab => tab.id === id))
                .filter(Boolean).map(renderTab)}
            </div>
          </section>
        ))}
      </div>
    </nav>
  );
}
