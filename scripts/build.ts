import {BuildUtils} from 'that-build-library';

BuildUtils
	.clean('dist/')
	.flatMap(() => BuildUtils.tsc('tsconfig.json'))
	.flatMap(() => BuildUtils.copy('README.md', 'dist'))
	.flatMap(() => BuildUtils.copy('LICENSE', 'dist'))
	.subscribe();
