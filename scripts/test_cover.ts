import {BuildUtils} from 'that-dev-library';

BuildUtils
	.exec('TESTING', 'nyc', [
		'--reporter', 'lcov',
		'--all', 'true',
		'--report-dir', './coverage',
		'--temp-directory', './coverage/tmp',
		'--exclude', 'src/**/*.spec.*',
		'--include', 'src/**/*',
		'node_modules/.bin/jasmine',
		'src/*.spec.js',
		'src/**/*.spec.js'
	])
	.flatMap(() => BuildUtils
		.exec('SENDING TO CODECOV', 'codecov', [])
	)
	.subscribe();
