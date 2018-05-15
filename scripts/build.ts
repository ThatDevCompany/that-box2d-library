import {BuildUtils} from 'that-dev-library/build';

BuildUtils
	.clean('dist/')
	.flatMap(() => BuildUtils.tsc('tsconfig.json'))
	.flatMap(() => BuildUtils.copy('README.md', 'dist'))
	.flatMap(() => BuildUtils.copy('LICENSE', 'dist'))
	.subscribe();
