import App from './src/app.js'
new App().open().catch(err => {
    console.error(err)
    process.exit(1)
})
