"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var that_build_library_1 = require("that-build-library");
that_build_library_1.BuildUtils
    .clean('dist/')
    .flatMap(function () { return that_build_library_1.BuildUtils.tsc('tsconfig.json'); })
    .flatMap(function () { return that_build_library_1.BuildUtils.copy('README.md', 'dist'); })
    .flatMap(function () { return that_build_library_1.BuildUtils.copy('LICENSE', 'dist'); })
    .subscribe();
