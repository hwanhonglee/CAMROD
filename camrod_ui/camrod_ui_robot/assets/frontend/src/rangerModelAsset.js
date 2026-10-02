// HH_261001 - Public assets keep stable filenames for CARLA/CAMROD packaging. npm embeds
// a content-derived revision so browsers fetch the latest model after a build.
const revision = process.env.REACT_APP_RANGER_ASSET_REV
  || (process.env.NODE_ENV === 'development' ? Date.now().toString(36) : 'unversioned');
const asset = filename => `/models/${filename}?v=${revision}`;

export const RANGER_MODEL_URL = asset('ranger-navigation.glb');
export const RANGER_SIDE_WRAP_URL = asset('woraksan-side-wrap.png');
export const RANGER_FRONT_WRAP_URL = asset('woraksan-front-wrap.png');
export const RANGER_REAR_WRAP_URL = asset('woraksan-rear-wrap.png');
