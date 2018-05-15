import {BuildUtils} from 'that-dev-library/build';

BuildUtils
	.exec('LINTING', 'tslint', ['-p', 'src/tsconfig.json', '--fix'])
	.subscribe();
