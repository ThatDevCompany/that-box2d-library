"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var that_build_library_1 = require("that-build-library");
that_build_library_1.BuildUtils
    .exec('TESTING', 'istanbul', [
    'cover',
    '--config',
    '.istanbul.yml',
    '--include-all-sources',
    'node_modules/.bin/jasmine',
    'src/*.spec.js',
    'src/**/*.spec.js'
])
    .subscribe();
