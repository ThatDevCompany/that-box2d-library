const BuildUtils = require('that-build-library').BuildUtils

module.exports = Promise.resolve()
	.then(() => BuildUtils.echo('CLEANING'))
	.then(() => BuildUtils.clean('dist'))

	.then(() => BuildUtils.echo('BUILDING'))
	.then(() => BuildUtils.tsc('tsconfig.prod.json'))

	.then(() => BuildUtils.echo('TIDYING'))
	.then(() => BuildUtils.removeModuleAlias('@', 'dist'))
	.then(() => BuildUtils.copy('README.md', 'dist'))
	.then(() => BuildUtils.copy('LICENSE', 'dist'))
