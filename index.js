const App = require('./src/app')
if (require.main === module) {
    new App().listen().catch(err => {
        console.error(err)
        process.exit(1)
    })
}