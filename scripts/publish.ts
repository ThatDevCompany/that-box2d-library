import {BuildUtils} from 'that-dev-library';

BuildUtils
	.npmPublish('dist', (pkg) => {
		delete pkg.scripts;
		pkg.main = 'index.js';
	})
	.subscribe();
