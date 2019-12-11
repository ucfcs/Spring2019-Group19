const Sequelize = require('sequelize');

const sequelize = new Sequelize('database', 'user', 'password', {
     host: 'localhost',
     dialect: 'sqlite',
     logging: false,
     storage: 'database.sqlite'


  pool: {
    max: 100,
    min: 0,
    acquire: 30000,
    idle: 1000,0
  }

});

const Maps = sequelize.define('maps', {
     name: {
        type: Sequelize.STRING,
        unique: true,
     },
     map: {
     	type: Sequelize.BLOB,
     }


});
