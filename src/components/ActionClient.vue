<template>
  <div class="base">
    <v-toolbar color="cyan" dark fixed app>
      <v-toolbar-title>Fibonacci client</v-toolbar-title>
      <v-spacer />
      <v-chip>
        <v-avatar :style="{'backgroundColor': connectionColor}"></v-avatar>
        {{ connectionStatus }}
      </v-chip>
    </v-toolbar>
    <div>Enter a number to generate a Fibonacci sequence of that order.</div>
    <div>Check out your results below!</div>
    <v-container>
      <v-card>
        <div>
          <v-card-title><h4>Trigger Fibonacci sequence</h4></v-card-title>
          <v-divider />
          <v-container>
            <v-text-field label="Order (example: 5)" v-model.number="order"></v-text-field>
            <v-btn small v-on:click="triggerAction">Trigger action</v-btn>
            <v-btn small v-on:click="cancelAction">Cancel action</v-btn>
          </v-container>
        </div>
      </v-card>
    </v-container>
    <v-container>
      <v-card width="500px">
        <v-card-title><h4>Results</h4></v-card-title>
        <v-divider />
        <v-list dense three-line>
          <v-list-tile>
            <v-list-tile-content>Feedback</v-list-tile-content>
            <v-list-tile-content class="align-end">
              <v-container id="scroll-target" style="max-height: 200px" class="scroll-y">
                {{ feedback }}
                <v-layout v-scroll:#scroll-target="onScroll" column align-center justify-center style="height: 200px"></v-layout>
              </v-container>
            </v-list-tile-content>
          </v-list-tile>
          <v-list-tile>
            <v-list-tile-content>Final result</v-list-tile-content>
            <v-list-tile-content class="align-end">
              <v-container id="scroll-target" style="max-height: 200px" class="scroll-y">
                {{ result }}
                <v-layout v-scroll:#scroll-target="onScroll" column align-center justify-center style="height: 200px"></v-layout>
              </v-container>
            </v-list-tile-content>
          </v-list-tile>
          <v-list-tile>
            <v-list-tile-content style="height: 300px">Status</v-list-tile-content>
            <v-list-tile-content class="align-left">{{ status }}</v-list-tile-content>
          </v-list-tile>
        </v-list>
      </v-card>
    </v-container>
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

      feedback: "",
      result: "",
      status: "Awaiting...",
    }
  },
  computed: {
    connectionColor() {
      return this.isConnected ? 'mediumSeaGreen' : 'tomato';
    }
  },
  methods: {
    resetResults() {
      this.feedback = "";
      this.result = "";
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
.base {
  font-family: sans-serif;
}
</style>
