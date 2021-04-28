const App = require('./src/app')
if (require.main === module) {
    new App().open().catch(err => {
        console.error(err)
        process.exit(1)
    })
}