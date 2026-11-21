import '@fontsource-variable/anybody/wdth.css';
import '@fontsource-variable/overpass/wght.css';
import './styles/tokens.css';
import './styles/app.css';
import { createRoot } from 'react-dom/client';
import { App } from './App.tsx';

// No StrictMode: its double-mount in dev would build the WebGL scene and
// fetch the 12 MB robot model twice.
createRoot(document.getElementById('root')!).render(<App />);
