;; Auto-generated. Do not edit!


(when (boundp 'movemaster_msg::status)
  (if (not (find-package "MOVEMASTER_MSG"))
    (make-package "MOVEMASTER_MSG"))
  (shadow 'status (find-package "MOVEMASTER_MSG")))
(unless (find-package "MOVEMASTER_MSG::STATUS")
  (make-package "MOVEMASTER_MSG::STATUS"))

(in-package "ROS")
;;//! \htmlinclude status.msg.html


(defclass movemaster_msg::status
  :super ros::object
  :slots (_joint _setpoint _pulse_count _error _output _control_loop _IsDone ))

(defmethod movemaster_msg::status
  (:init
   (&key
    ((:joint __joint) "")
    ((:setpoint __setpoint) 0.0)
    ((:pulse_count __pulse_count) 0.0)
    ((:error __error) 0.0)
    ((:output __output) 0.0)
    ((:control_loop __control_loop) 0.0)
    ((:IsDone __IsDone) nil)
    )
   (send-super :init)
   (setq _joint (string __joint))
   (setq _setpoint (float __setpoint))
   (setq _pulse_count (float __pulse_count))
   (setq _error (float __error))
   (setq _output (float __output))
   (setq _control_loop (float __control_loop))
   (setq _IsDone __IsDone)
   self)
  (:joint
   (&optional __joint)
   (if __joint (setq _joint __joint)) _joint)
  (:setpoint
   (&optional __setpoint)
   (if __setpoint (setq _setpoint __setpoint)) _setpoint)
  (:pulse_count
   (&optional __pulse_count)
   (if __pulse_count (setq _pulse_count __pulse_count)) _pulse_count)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:control_loop
   (&optional __control_loop)
   (if __control_loop (setq _control_loop __control_loop)) _control_loop)
  (:IsDone
   (&optional (__IsDone :null))
   (if (not (eq __IsDone :null)) (setq _IsDone __IsDone)) _IsDone)
  (:serialization-length
   ()
   (+
    ;; string _joint
    4 (length _joint)
    ;; float32 _setpoint
    4
    ;; float32 _pulse_count
    4
    ;; float32 _error
    4
    ;; float32 _output
    4
    ;; float32 _control_loop
    4
    ;; bool _IsDone
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _joint
       (write-long (length _joint) s) (princ _joint s)
     ;; float32 _setpoint
       (sys::poke _setpoint (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pulse_count
       (sys::poke _pulse_count (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _error
       (sys::poke _error (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _output
       (sys::poke _output (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _control_loop
       (sys::poke _control_loop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _IsDone
       (if _IsDone (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _joint
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _joint (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _setpoint
     (setq _setpoint (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pulse_count
     (setq _pulse_count (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _error
     (setq _error (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _output
     (setq _output (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _control_loop
     (setq _control_loop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _IsDone
     (setq _IsDone (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get movemaster_msg::status :md5sum-) "283025d1e58370fa7a43c23c572de06b")
(setf (get movemaster_msg::status :datatype-) "movemaster_msg/status")
(setf (get movemaster_msg::status :definition-)
      "string joint
float32 setpoint
float32 pulse_count
float32 error
float32 output
float32 control_loop
bool IsDone
")



(provide :movemaster_msg/status "283025d1e58370fa7a43c23c572de06b")


