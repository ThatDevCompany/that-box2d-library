import {BuildUtils} from 'that-dev-library';

BuildUtils
	.exec('LINTING', 'tslint', ['-p', 'src/tsconfig.json', '--fix'])
	.subscribe();
