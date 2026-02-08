import App from './src/app.js'
console.log('saldjfhaskjdhf')
new App().open().catch(err => {
    console.error(err)
    process.exit(1)
})
