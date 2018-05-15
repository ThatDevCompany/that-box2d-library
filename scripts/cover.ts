import {BuildUtils} from 'that-dev-library/build';

BuildUtils
	.exec('SENDING TO CODECOV', 'codecov', [])
	.subscribe();
