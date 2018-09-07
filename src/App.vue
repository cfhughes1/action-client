<template>
  <v-app id="inspire">
    <div id="app">
      <v-container fluid fill-height>
        <v-layout align-center justify-center>
          <ActionClient />
        </v-layout>
      </v-container>
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
