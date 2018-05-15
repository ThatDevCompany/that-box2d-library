import {BuildUtils} from 'that-dev-library';

BuildUtils
	.exec('SENDING TO CODECOV', 'codecov', [])
	.subscribe();
