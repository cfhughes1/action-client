<template>
  <v-app id="inspire">
    <v-toolbar color="cyan" dark fixed app>
      <v-toolbar-title>Fibonacci Client</v-toolbar-title>
    </v-toolbar>
    <div id="app">
      <ActionClient />
    </div>
    <v-footer color="cyan" app>
    </v-footer>
  </v-app>
</template>

<script>
import ActionClient from './components/ActionClient.vue'

export default {
  name: 'app',
  components: {
    ActionClient
  }
}
</script>

<style>
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
</style>
