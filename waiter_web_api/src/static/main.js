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
        viewer: null,
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
                this.setTopic()
                this.setCamera()
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                document.getElementById('divCamera').innerHTML = ''
            })
        },
        disconnect: function() {
            this.ros.close()
        },

        
        //------------------------------------//
        // -------------Service---------------//
        //------------------------------------//

        setTopic: function() {
            this.home_pub = new ROSLIB.Topic({
                ros: this.ros,
                name: this.topic_prefix+'/behavior_state/home_command',
                messageType: 'std_msgs/Bool'
            })

            this.serve_pub = new ROSLIB.Topic({
                ros: this.ros,
                name: this.topic_prefix+'/behavior_state/serve_command',
                messageType: 'std_msgs/Bool'
            })

            this.abort_pub = new ROSLIB.Topic({
                ros: this.ros,
                name: this.topic_prefix+'/behavior_state/abort_command',
                messageType: 'std_msgs/Bool'
            })

        },
        
        abort: function() {
            console.log("abort");
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: this.topic_prefix+'/behavior_service/set_abort',
                serviceType: 'std_srvs/SetBool',
            });
            
            let request = new ROSLIB.ServiceRequest({data:true});
            
            service.callService(request, function(result){
                console.log(result)
            });
            
            //document.getElementById('serve_btn').disabled = false
            //document.getElementById('home_btn').disabled = false
            //document.getElementById('abort_btn').disabled = true
            
        },
        
        home: function() {
            console.log("home");
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: this.topic_prefix+'/behavior_service/set_home',
                serviceType: 'std_srvs/SetBool',
            });
            
            let request = new ROSLIB.ServiceRequest({data:true});
            
            service.callService(request, function(result){
                console.log(result)
            });
            
            //document.getElementById('serve_btn').disabled = true
            //document.getElementById('home_btn').disabled = true
            //document.getElementById('abort_btn').disabled = false
        },
        
        serve: function() {
            console.log("serve")
            let service = new ROSLIB.Service({
                ros: this.ros,
                name: this.topic_prefix+'/behavior_service/set_serve',
                serviceType: 'std_srvs/SetBool',
            });
            
            let request = new ROSLIB.ServiceRequest({data:true});
            
            service.callService(request, function(result){
                console.log(result)
            });
            
            //document.getElementById('serve_btn').disabled = true
            //document.getElementById('home_btn').disabled = true
            //document.getElementById('abort_btn').disabled = false
        },
        
        
//changeStream
        setCamera: function() {
            let host = '0.0.0.0:8080'
            //let host = '192.168.12.254:8080'
            console.log("global")
            this.viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 640,
                height: 480,
                //topic: '/usb_cam/image_raw&type=ros_compressed',
                topic: this.topic_prefix+'/global_tracking/image_raw',
                ssl: true,
            })
        },
        
        setGlobal: function(){
            this.viewer.changeStream(this.topic_prefix+'/global_tracking/image_raw')
        
        },
        
        setLocal: function(){
            this.viewer.changeStream(this.topic_prefix+'/local_tracking/image_raw')
        
        },

    },
    mounted() {
        this.connect()
    },
})
