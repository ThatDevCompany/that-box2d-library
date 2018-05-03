"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
var that_build_library_1 = require("that-build-library");
that_build_library_1.BuildUtils
    .exec('LINTING', 'tslint', ['-p', 'src/tsconfig.json', '--fix'])
    .subscribe();
