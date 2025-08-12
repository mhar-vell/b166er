;; Auto-generated. Do not edit!


(when (boundp 'movemaster_msg::setpoint)
  (if (not (find-package "MOVEMASTER_MSG"))
    (make-package "MOVEMASTER_MSG"))
  (shadow 'setpoint (find-package "MOVEMASTER_MSG")))
(unless (find-package "MOVEMASTER_MSG::SETPOINT")
  (make-package "MOVEMASTER_MSG::SETPOINT"))

(in-package "ROS")
;;//! \htmlinclude setpoint.msg.html


(defclass movemaster_msg::setpoint
  :super ros::object
  :slots (_set_1 _set_2 _set_3 _set_4 _set_5 _set_GRIP _emergency_stop _GoHome ))

(defmethod movemaster_msg::setpoint
  (:init
   (&key
    ((:set_1 __set_1) 0.0)
    ((:set_2 __set_2) 0.0)
    ((:set_3 __set_3) 0.0)
    ((:set_4 __set_4) 0.0)
    ((:set_5 __set_5) 0.0)
    ((:set_GRIP __set_GRIP) nil)
    ((:emergency_stop __emergency_stop) nil)
    ((:GoHome __GoHome) 0)
    )
   (send-super :init)
   (setq _set_1 (float __set_1))
   (setq _set_2 (float __set_2))
   (setq _set_3 (float __set_3))
   (setq _set_4 (float __set_4))
   (setq _set_5 (float __set_5))
   (setq _set_GRIP __set_GRIP)
   (setq _emergency_stop __emergency_stop)
   (setq _GoHome (round __GoHome))
   self)
  (:set_1
   (&optional __set_1)
   (if __set_1 (setq _set_1 __set_1)) _set_1)
  (:set_2
   (&optional __set_2)
   (if __set_2 (setq _set_2 __set_2)) _set_2)
  (:set_3
   (&optional __set_3)
   (if __set_3 (setq _set_3 __set_3)) _set_3)
  (:set_4
   (&optional __set_4)
   (if __set_4 (setq _set_4 __set_4)) _set_4)
  (:set_5
   (&optional __set_5)
   (if __set_5 (setq _set_5 __set_5)) _set_5)
  (:set_GRIP
   (&optional (__set_GRIP :null))
   (if (not (eq __set_GRIP :null)) (setq _set_GRIP __set_GRIP)) _set_GRIP)
  (:emergency_stop
   (&optional (__emergency_stop :null))
   (if (not (eq __emergency_stop :null)) (setq _emergency_stop __emergency_stop)) _emergency_stop)
  (:GoHome
   (&optional __GoHome)
   (if __GoHome (setq _GoHome __GoHome)) _GoHome)
  (:serialization-length
   ()
   (+
    ;; float32 _set_1
    4
    ;; float32 _set_2
    4
    ;; float32 _set_3
    4
    ;; float32 _set_4
    4
    ;; float32 _set_5
    4
    ;; bool _set_GRIP
    1
    ;; bool _emergency_stop
    1
    ;; int8 _GoHome
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _set_1
       (sys::poke _set_1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _set_2
       (sys::poke _set_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _set_3
       (sys::poke _set_3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _set_4
       (sys::poke _set_4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _set_5
       (sys::poke _set_5 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _set_GRIP
       (if _set_GRIP (write-byte -1 s) (write-byte 0 s))
     ;; bool _emergency_stop
       (if _emergency_stop (write-byte -1 s) (write-byte 0 s))
     ;; int8 _GoHome
       (write-byte _GoHome s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _set_1
     (setq _set_1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _set_2
     (setq _set_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _set_3
     (setq _set_3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _set_4
     (setq _set_4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _set_5
     (setq _set_5 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _set_GRIP
     (setq _set_GRIP (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _emergency_stop
     (setq _emergency_stop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _GoHome
     (setq _GoHome (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _GoHome 127) (setq _GoHome (- _GoHome 256)))
   ;;
   self)
  )

(setf (get movemaster_msg::setpoint :md5sum-) "aaa39b4215432c08ac9b36247ded9610")
(setf (get movemaster_msg::setpoint :datatype-) "movemaster_msg/setpoint")
(setf (get movemaster_msg::setpoint :definition-)
      "float32 set_1
float32 set_2
float32 set_3
float32 set_4
float32 set_5
bool 	set_GRIP
bool 	emergency_stop
int8 	GoHome
")



(provide :movemaster_msg/setpoint "aaa39b4215432c08ac9b36247ded9610")


