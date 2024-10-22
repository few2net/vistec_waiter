// ws://54.147.26.233:9090

var app = new Vue({
    el: '#app',
    // computed values
    computed: {
        ws_address: function() {
            return `${this.rosbridge_address}`
        },
    },
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        topic: null,
        message: null,
        rosbridge_address: 'ws://0.0.0.0:9090',
        //rosbridge_address: 'ws://192.168.12.254:9090',
        port: '9090',
        topic_prefix: "/hook/camera",
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.ws_address
            })
            console.log(this.ws_address)
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
            })
        },
        disconnect: function() {
            this.ros.close()
        },

        //------------------------------------//
        // -------------Service---------------//
        //------------------------------------//

        submitService: function() {
            console.log("service called")
            // service is busy
            this.service_busy = true
            this.service_response = ''
            // define the service to be called
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: this.topic_prefix+'/updateSVM',
                serviceType: 'std_srvs/Trigger',
            })

            // define the request
            let request = new ROSLIB.ServiceRequest({})

            // define a callback
            service.callService(request, (result) => {
                this.service_busy = false
                this.service_response = JSON.stringify(result)
            })
        },

    },
    mounted() {
        this.connect()
    },
})
