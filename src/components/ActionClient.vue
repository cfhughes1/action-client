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
      fibonacciClient: null,
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
      return this.isConnected ? 'green' : 'red';
    },
    triggerAction: function(event) {
      this.result = "Awaiting...";

      this.goal = new this.ROSLIB.Goal({
        actionClient : this.fibonacciClient,
        goalMessage : {
          order : parseInt(this.order),
        }
      });

      this.goal.on('result', function(result) {
        this.result = result.sequence;
        console.log('Final Result: ' + result.sequence);
      }.bind(this));

      this.goal.on('feedback', function(feedback) {
        this.feedback = feedback.sequence;
        console.log('Feedback: ' + feedback.sequence);
      }.bind(this));

      this.goal.send();
    },
  },
  mounted() {
    this.ROSLIB = require('roslib');
    this.ros = new this.ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });
    this.fibonacciClient = new this.ROSLIB.ActionClient({
      ros : this.ros,
      serverName : '/fibonacci',
      actionName : 'actionlib_tutorials/FibonacciAction'
    });

  },
  watch: {
    ros() {
      this.ros.on('connection', function() {
        this.connectionStatus = 'Connected';
        this.isConnected = true;
        console.log('Connected to websocket server.');
      }.bind(this));

      this.ros.on('error', function(error) {
        this.connectionStatus = 'Error connecting';
        console.log('Error connecting to websocket server: ', error);
      }.bind(this));

      this.ros.on('close', function() {
        this.connectionStatus = 'Connection closed';
        this.isConnected = false;
        console.log('Connection to websocket server closed.');
      }.bind(this));

    }
  }
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
