const BuildUtils = require('that-build-library').BuildUtils

module.exports = Promise.resolve()
	.then(() => BuildUtils.echo('PUBLISHING'))
	.then(() =>
		BuildUtils.publish('dist', pkg => {
			delete pkg.scripts
			delete pkg.devDependencies
		})
	)
	.catch(e => {
		console.error('An error occurred during unit testing', e)
		process.exit(1)
	})
