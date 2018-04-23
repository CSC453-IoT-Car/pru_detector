var b = require('./detector.js');
b.enable();
var interval = setInterval(function f() {
console.log(b.getLastReading(0));
}, 500);
