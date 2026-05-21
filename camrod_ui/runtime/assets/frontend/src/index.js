/**
 * index.js — React 애플리케이션의 진입점(Entry Point)
 *
 * 역할:
 *   - HTML의 <div id="root"> 요소를 찾아 React 앱을 마운트(렌더링)
 *   - App 컴포넌트를 최상위로 렌더링
 */

import React from 'react';               // React 코어 라이브러리
import ReactDOM from 'react-dom/client'; // React 18 DOM 렌더링 API
import App from './App';                 // 메인 App 컴포넌트 (토글 버튼 UI)

// HTML에서 id="root"인 DOM 요소를 React 루트로 생성
const root = ReactDOM.createRoot(document.getElementById('root'));

// App 컴포넌트를 루트에 렌더링 → 화면에 토글 버튼 UI가 표시됨
root.render(<App />);
