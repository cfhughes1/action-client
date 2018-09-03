<template>
  <div>
    <button v-on:click="triggerAction">Trigger action</button>
    <div>Connection status: {{ connectionStatus }}</div>
    <div>Feedback: {{ feedback }}</div>
    <div>Result: {{ result }}</div>
  </div>
</template>

<script>

export default {
  name: 'ActionClient',
  data() {
    return {
      ros: null,
      goal: null,
      feedback: null,
      result: null,
      connectionStatus: null,
    }
  },
  methods: {
    triggerAction: function(event) {
      console.log("action triggered");
      this.$data.goal.send();
    },
  },
  mounted() {
    var ROSLIB = require('roslib');
    this.$data.ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });
    var fibonacciClient = new ROSLIB.ActionClient({
      ros : this.$data.ros,
      serverName : '/fibonacci',
      actionName : 'actionlib_tutorials/FibonacciAction'
    });
    this.$data.goal = new ROSLIB.Goal({
      actionClient : fibonacciClient,
      goalMessage : {
        order : 5,
      }
    });

    this.$data.goal.on('result', function(result) {
      this.$data.result = result.sequence;
      console.log('Final Result: ' + result.sequence);
    }.bind(this));

    this.$data.goal.on('feedback', function(feedback) {
      this.$data.feedback = feedback.sequence;
      console.log('Feedback: ' + feedback.sequence);
    }.bind(this));

    this.$data.ros.on('connection', function() {
      this.$data.connectionStatus = 'Connected';
      console.log('Connected to websocket server.');
    }.bind(this));

    this.$data.ros.on('error', function(error) {
      this.$data.connectionStatus = 'Error connecting';
      console.log('Error connecting to websocket server: ', error);
    }.bind(this));

    this.$data.ros.on('close', function() {
      this.$data.connectionStatus = 'Connection closed';
      console.log('Connection to websocket server closed.');
    }.bind(this));
  },
}
</script>

<style scoped>
h3 {
  margin: 40px 0 0;
}
ul {
  list-style-type: none;
  padding: 0;
}
li {
  display: inline-block;
  margin: 0 10px;
}
a {
  color: #42b983;
}
</style>
