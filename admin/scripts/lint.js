const BuildUtils = require('that-build-library').BuildUtils

module.exports = Promise.resolve()
	.then(() => BuildUtils.echo('Prettiering Admin Javascript'))
	.then(() => BuildUtils.prettierJS('admin'))

	.then(() => BuildUtils.echo('Prettiering SRC Typescript'))
	.then(() => BuildUtils.prettierTS('src'))

	.catch(e => {
		console.error('An error occurred', e)
		process.exit(1)
	})
