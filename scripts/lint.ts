import {BuildUtils} from 'that-build-library';

BuildUtils
	.exec('LINTING', 'tslint', ['-p', 'src/tsconfig.json', '--fix'])
	.subscribe();
