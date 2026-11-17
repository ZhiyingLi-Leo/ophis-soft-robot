# Three.js 本地运行依赖

来源：https://github.com/mrdoob/three.js/tree/r170 ，版本0.170.0，MIT许可见 LICENSE.three。

保留完整的 three.module.js、GLTFLoader.js、OrbitControls.js、BufferGeometryUtils.js、RoomEnvironment.js。GLTFLoader中BufferGeometryUtils导入路径调整为同目录；其余实现未修改。浏览器通过import map将 `three` 指向本地模块，无CDN请求。

官方接口参考：https://threejs.org/docs/pages/GLTFLoader.html 。
