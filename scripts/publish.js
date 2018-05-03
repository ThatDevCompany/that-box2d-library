"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var that_build_library_1 = require("that-build-library");
that_build_library_1.BuildUtils
    .npmPublish('dist', function (pkg) {
    delete pkg.scripts;
    pkg.main = 'index.js';
})
    .subscribe();
