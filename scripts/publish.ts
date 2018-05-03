import {BuildUtils} from 'that-build-library';

BuildUtils
	.npmPublish('dist', (pkg) => {
		delete pkg.scripts;
		pkg.main = 'index.js';
	})
	.subscribe();
