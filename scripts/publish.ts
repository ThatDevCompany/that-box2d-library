import {BuildUtils} from 'that-build-library';

BuildUtils
	.npmPublish('dist', (pkg) => {
		delete pkg.scripts;
	})
	.subscribe();
