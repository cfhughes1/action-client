<template>
  <div class="base">
      <div class="status-col">
        <div class="header">Connection status</div>
          <div class="connected-status-light" :style="{ 'backgroundColor' : getConnectionColor() }">
          <span style="padding-left: 30px">{{ connectionStatus }}</span>
        </div>
      </div>
      <div class="status-col">
        <div class="header">Trigger Fibonacci sequence</div>
        <input v-model.number="order" placeholder="order">
        <button v-on:click="triggerAction">Trigger action</button>
        <button v-on:click="cancelAction">Cancel</button>
      </div>
      <div class="status-col">
        <div class="header">Results</div>
        <div>Feedback: {{ feedback }}</div>
        <div>Final result: {{ result }}</div>
        <div>Status: {{ status }}</div>
      </div>
  </div>
</template>

<script>
const StatusEnum = Object.freeze({
  0: "Pending",
  1: "Active",
  2: "Preempted",
  3: "Succeeded",
  4: "Aborted",
  5: "Rejected",
  6: "Preempting",
  7: "Recalling",
  8: "Recalled",
  9: "Lost"
});

export default {
  name: 'ActionClient',
  data() {
    return {
      ROSLIB: null,
      ros: null,
      fibonacciClient: null,
      goal: null,

      connectionStatus: "Initializing",
      isConnected: false,
      order: null,

      feedback: "Awaiting...",
      result: "Awaiting...",
      status: "Awaiting...",
    }
  },
  methods: {
    getConnectionColor() {
      return this.isConnected ? 'green' : 'red';
    },

    resetResults() {
      this.feedback = "Awaiting...";
      this.result = "Awaiting...";
    },

    cancelAction() {
      this.goal.cancel();
    },

    triggerAction() {
      this.resetResults();

      this.goal = new this.ROSLIB.Goal({
        actionClient : this.fibonacciClient,
        goalMessage : {
          order : this.order,
        }
      });

      this.goal.on('status', function(result) {
        this.status = StatusEnum[result.status];
      }.bind(this));

      this.goal.on('result', function(result) {
        this.result = result.sequence;
        // console.log('Final Result: ' + result.sequence);
      }.bind(this));

      this.goal.on('feedback', function(feedback) {
        this.feedback = feedback.sequence;
        // console.log('Feedback: ' + feedback.sequence);
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
        // console.log('Connected to websocket server.');
      }.bind(this));

      this.ros.on('error', function(error) {
        this.connectionStatus = 'Error connecting: {}'.replace("{}", error);
        // console.log('Error connecting to websocket server: ', error);
      }.bind(this));

      this.ros.on('close', function() {
        this.connectionStatus = 'Connection closed';
        this.isConnected = false;
        // console.log('Connection to websocket server closed.');
      }.bind(this));

    }
  }
}
</script>

<style scoped>
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

.connected-status-light {
  border-radius: 50%;
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
