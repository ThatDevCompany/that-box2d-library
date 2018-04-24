import {BuildUtils} from 'that-build-library';

BuildUtils
	.exec('TESTING', 'istanbul', [
		'cover',
		'--config',
		'.istanbul.yml',
		'--include-all-sources',
		'node_modules/.bin/jasmine',
		'src/*.spec.js',
		'src/**/*.spec.js'
	])
	.subscribe();
