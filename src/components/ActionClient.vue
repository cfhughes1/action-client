<template>
  <div class="base">
      <div class="status-col">
        <div class="header">Connection status</div>
          <div class="connected" :style="{ 'backgroundColor' : getConnectionColor() }">
          <span style="padding-left: 30px;">{{ connectionStatus }}</span>
        </div>
      </div>
      <div class="status-col">
        <div class="header">Trigger Fibonacci sequence</div>
        <input v-model="order" placeholder="order">
        <button v-on:click="triggerAction">Trigger action</button>
      </div>
      <div class="status-col">
        <div class="header">Results</div>
        <div>Feedback: {{ feedback }}</div>
        <div>Final result: {{ result }}</div>
      </div>
  </div>
</template>

<script>
export default {
  name: 'ActionClient',
  data() {
    return {
      ROSLIB: null,
      ros: null,
      goal: null,
      feedback: "Awaiting...",
      result: "Awaiting...",
      connectionStatus: "Initializing",
      isConnected: false,
      order: null,
    }
  },
  methods: {
    getConnectionColor() {
      return this.$data.isConnected ? 'green' : 'red';
    },
    triggerAction: function(event) {
      this.$data.result = "Awaiting...";
      this.$data.goal.send();
    },
  },
  mounted() {
    this.$data.ROSLIB = require('roslib');
    this.$data.ros = new this.$data.ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });
    var fibonacciClient = new this.$data.ROSLIB.ActionClient({
      ros : this.$data.ros,
      serverName : '/fibonacci',
      actionName : 'actionlib_tutorials/FibonacciAction'
    });

    this.$data.goal = new this.$data.ROSLIB.Goal({
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
      this.$data.isConnected = true;
      console.log('Connected to websocket server.');
    }.bind(this));

    this.$data.ros.on('error', function(error) {
      this.$data.connectionStatus = 'Error connecting';
      console.log('Error connecting to websocket server: ', error);
    }.bind(this));

    this.$data.ros.on('close', function() {
      this.$data.connectionStatus = 'Connection closed';
      this.$data.isConnected = false;
      console.log('Connection to websocket server closed.');
    }.bind(this));
  },
}
</script>

<style scoped>
.status-row {

}

.status-col {
  width: 30%;
  float: left;
  text-align: left;
  padding-left: 20px;
  height: 100px;
}

.status-col:not(:first-child) {
  border-left: 1px solid;
}

.header {
  font-weight: bold;
  padding-bottom: 10px;
}

.connected {
  border-radius: 50%;
  //background-color: green;
  height: 20px;
  width: 20px;
  display: inline-block;
  float: left;
}

.base {
  font-family: sans-serif;
}

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
