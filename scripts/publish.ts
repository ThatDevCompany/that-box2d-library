import {BuildUtils} from 'that-dev-library/build';

BuildUtils
	.npmPublish('dist', (pkg) => {
		delete pkg.scripts;
		pkg.main = 'index.js';
	})
	.subscribe();
