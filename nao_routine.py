import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import json
import os
import qi
import sys
import math
import google.generativeai as genai

class NaoTTSNode(Node):
    def __init__(self):
        super().__init__('nao_tts_node')

        # Parametri connessione NAO
        self.declare_parameter('ip', '192.168.0.101')
        self.declare_parameter('port', 9559)
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        # Connessione sessione NAOqi
        self.session = self.connect_nao(ip, port)

        self.motion_service = self.session.service("ALMotion")
        self.life_service = self.session.service("ALAutonomousLife")
        self.posture_service = self.session.service("ALRobotPosture")
        self.awareness_service = self.session.service("ALBasicAwareness")
        self.memory_service = self.session.service("ALMemory")
        self.tts = self.session.service("ALTextToSpeech")
        self.tts.setLanguage("Italian")


        self.actual_pose_id = 1
        self.load_poses()
        self.fix_position()
        
        self.subscriber2 = self.memory_service.subscriber("ALTextToSpeech/Status")
        self.subscriber2.signal.connect(self.on_status)
        
        self.tts.say(f"Ciao! Iniziamo con la posa {self.poses[self.actual_pose_id]['name']}")
        
        self.apply_pose(pose_id=self.actual_pose_id)

        self.frames_failed = 0
        self.failed_pose_count = 0
        self.last_quote = None
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/nao_pose_info',
            self.check_angles_callback,
            10
        )
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer2 = self.create_timer(10.0, self.timer_llm)
        
    def on_status(self, value):
            print("Evento parlato:", value[1])
            if value[1] == "done":
                self.timer2.reset()


    def query_gemini(self):
        model = genai.GenerativeModel("gemini-1.5-pro")
        genai.configure(api_key="YOUR_API_KEY")
        print(self.differences)
        
        explanations = ""
        angles_needed = [False, False, False]
        angles_explained = [
            """
            - Se ShoulderRoll ha un valore positivo, significa che il braccio è troppo vicino al corpo (punta internamente) e deve essere allontanato dal corpo.
            - Se ShoulderRoll ha un valore negativo, significa che il braccio è troppo lontano dal corpo (punta esternamente) e deve essere avvicinato al corpo.
            """,
            """
            - Se ShoulderPitch ha un valore positivo, significa che il braccio è troppo basso e deve essere alzato.
            - Se ShoulderPitch ha un valore negativo, significa che il braccio è troppo alto e deve essere abbassato.
            """,
            """
            - Se ElbowRoll ha un valore positivo, significa che il gomito è troppo chiuso e deve essere aperto.
            - Se ElbowRoll ha un valore negativo, significa che il gomito è troppo aperto e deve essere chiuso.
            """

        ]

        for key in self.differences:
            if "ShoulderRoll" in key:
                angles_needed[0] = True
                if "LShoulderRoll" in key and not self.poses[self.actual_pose_id]['orientation'] == "lateralmente":
                    self.differences["LShoulderRoll"] = -self.differences["LShoulderRoll"]
            if "ShoulderPitch" in key:
                angles_needed[1] = True
            if "ElbowRoll" in key:
                angles_needed[2] = True
                if "LElbowRoll" in key and not self.poses[self.actual_pose_id]['orientation'] == "lateralmente":
                    self.differences["LElbowRoll"] = -self.differences["LElbowRoll"]
        
        if self.poses[self.actual_pose_id]['orientation'] == "lateralmente":
            angles_explained[0] = """
            - Se ShoulderRoll ha un valore positivo, significa che il braccio è troppo basso e deve essere alzato.
            - Se ShoulderRoll ha un valore negativo, significa che il braccio è troppo alto e deve essere abbassato.
            """
            angles_explained[1] = """
            - Se ShoulderPitch ha un valore positivo, significa che il braccio è troppo basso al corpo (punta internamente) e deve essere allontanato dal corpo.
            - Se ShoulderPitch ha un valore negativo, significa che il braccio è troppo alto dal corpo (punta esternamente) e deve essere avvicinato al corpo.
            """
        
        angles = "".join([f"--- {key}: {value}\n" for key, value in self.differences.items()])
        
        for i in range(len(angles_needed)):
            if angles_needed[i]:
                explanations += angles_explained[i]
                
        prompt = f"""
        Sei un fisioterapista virtuale che osserva la postura delle braccia di un paziente.  
        Ti fornisco i seguenti valori in radianti che descrivono la distanza di spalle e gomiti tra la posizione dell'utente e la posizione corretta:

        {angles}
        
        Ti faccio qualche esempio di come interpretare questi valori:
        
        {explanations}
        
        Fornisci suggerimenti naturali e chiari, di massimo un paio di frasi, per correggere la postura dell'utente in base agli errori rilevati.
        Rispondi in italiano.
        Esempio di risposta: "Abbassa leggermente il braccio sinistro", "Alza di molto il braccio destro", "Avvicina il braccio sinistro al corpo", "Allontana il braccio destro dal corpo", "Fletti completamente il gomito sinistro", "Chiudi il gomito destro".
        Evita di usare la parola "spalla" nelle tue risposte, usa sempre "braccio".
        Se non ci sono errori, non dire nulla.
        """
    
        if self.poses[self.actual_pose_id]['orientation'] == "lateralmente":
            angles_explained[0] = """
            - Se ShoulderRoll ha un valore positivo, significa che il braccio è troppo basso e deve essere alzato.
            - Se ShoulderRoll ha un valore negativo, significa che il braccio è troppo alto e deve essere abbassato.
            """
            angles_explained[1] = """
            - Se ShoulderPitch ha un valore positivo, significa che il braccio è troppo basso e deve essere alzato.
            - Se ShoulderPitch ha un valore negativo, significa che il braccio è troppo alto e deve essere abbassato.
            """
            
        print(prompt)
        response = model.generate_content(prompt, generation_config={"temperature":0, "max_output_tokens":100})

        self.timer2.cancel()
        
        return response.text



    def connect_nao(self, ip, port):
        session = qi.Session()
        try:
            session.connect(f"tcp://{ip}:{port}")
        except RuntimeError:
            self.get_logger().error(f"Impossibile connettersi a NAO all'IP {ip} sulla porta {port}.")
            sys.exit(1)
        return session
    
    def timer_callback(self):
        if hasattr(self, "advance") and self.advance:
            self.advance = False
            next_pose_id = self.actual_pose_id + 1
            if next_pose_id < len(self.poses):
                self.tts.say(f"Perfetto, ora facciamo la posa {self.poses[next_pose_id]['name']}")
                self.apply_pose(pose_id=next_pose_id)
            else:
                self.tts.say("Ottimo lavoro, spero tu ti senta meglio.")
                self.clap_hands()
                self.posture_service.goToPosture("Crouch", 0.5)
                self.destroy_node()
                rclpy.shutdown()

    def clap_hands(self):
        self.motion_service.setAngles(["LHand", "RHand"], [1.0, 1.0], 0.2)
        time.sleep(0.2)
        names = ["LShoulderRoll", "LShoulderPitch", "LElbowRoll","LElbowYaw","RShoulderRoll", "RShoulderPitch","RElbowRoll","RElbowYaw"]
        
        for _ in range(3):
            angles = [-0.5, 1.5, -1.5, -1.5, 0.5, 1.5, 1.5, 1.5]
            self.motion_service.setAngles(names, angles, 0.2)
            time.sleep(0.2)
            angles = [-0.5, 1.5, -1.5, -1.0, 0.5, 1.5, 1.5, 1.0]
            self.motion_service.setAngles(names, angles, 0.2)
            time.sleep(0.2)
        
        angles = [-0.5, 1.5, -1.5, -1.5, 0.5, 1.5, 1.5, 1.5]
        self.motion_service.setAngles(names, angles, 0.2)
        self.motion_service.setAngles(["LHand", "RHand"], [0.0, 0.0], 0.2)
        time.sleep(0.2)

    def timer_llm(self):
        if hasattr(self,'differences') and hasattr(self,'actual_orientation'):
            quote = self.query_gemini()
            self.get_logger().info(f"Gemini suggerisce: {quote}")
            self.tts.say(quote)
        else : 
            self.tts.say(f"c'é nessuno? Ho paura mi sento solo...")
                
    def check_angles_callback(self, msg: Float32MultiArray):
        arr = msg.data
        
        actual_orientation_val = arr.pop(0)  # Rimuovi e salva il primo elemento (orientamento)
        arr = [arr[self.actual_pose_angles_names.index(name)] if name in self.actual_pose_angles_names else 0.0 for name in self.actual_pose_angles_names]
        print(f"Ricevuti angoli: {arr}")
       
        if actual_orientation_val == 0.0:
            orientation = "di fronte"
        elif actual_orientation_val == 1.0:
            orientation = "lateralmente"
        elif actual_orientation_val == 2.0:
            orientation = "girato di spalle"

        self.actual_orientation = orientation

        if not hasattr(self, "actual_pose_angles"):
            return
        print(f"Angoli attuali: {self.actual_pose_angles}")
        
        # Se orientation è lateralmente, gli angoli vengono valutati in modulo
        if self.poses[self.actual_pose_id]['orientation'] == "lateralmente":
            arr = [abs(angle) for angle in arr]
            self.actual_pose_angles = [abs(angle) for angle in self.actual_pose_angles]
        
        tolerance = 0.3  # tolleranza in radianti
        all_within_tolerance = all(
            abs(arr[i] - self.actual_pose_angles[i]) <= tolerance for i in range(6)
        )
        
        # Controlla ogni angolo e stampa il risultato
        self.differences = {}
        for i in range(len(arr)):
            delta = arr[i] - self.actual_pose_angles[i]
            if abs(delta) > tolerance:
                self.differences[self.actual_pose_angles_names[i]] = round(delta, 2)

                self.get_logger().warning(f"Angolo {self.actual_pose_angles_names[i]} fuori tolleranza: {arr[i]:.2f} - {self.actual_pose_angles[i]:.2f}")
        self.get_logger().info(f"------------------------------------------------")

        try:
            text = ""
            for i in range(len(arr)):
                text += f"Angolo {self.actual_pose_angles_names[i]}: {arr[i]:.2f} - {self.actual_pose_angles[i]:.2f}\n"
            text += f"Tutti entro la tolleranza: {'Sì' if all_within_tolerance else 'No'}"
            self.advance = all_within_tolerance

            

        except Exception as e:
            self.get_logger().error(f"Errore TTS: {e}")
        if self.orientation is not None:
            self.get_logger().info(f"Orientamento rilevato: {orientation}, richiesto: {self.poses[self.actual_pose_id]['orientation']}")
            if orientation != self.orientation:
                if self.orientation == "lateralmente":
                    self.tts.say(f"Per favore, dammi il profilo destro.")
                else:
                    self.tts.say(f"Per favore, posizionati {self.orientation} a me.")
                self.advance = False
                time.sleep(2)

    def fix_position(self):
        self.life_service.setState("disabled")
        self.motion_service.setStiffnesses("Body", 1.0)
        self.posture_service.goToPosture("StandInit", 0.8)
        self.motion_service.setStiffnesses("Head", 0.0)
        self.awareness_service.setEnabled(False)

        names = ["HeadYaw", "HeadPitch"]
        angles = [0.0, 0.0]   # posizione neutra (dritto avanti)
        self.motion_service.setAngles(names, angles, 0.2)
    
    def load_poses(self, json_path="PATH_OF_poses.json"):
        with open(json_path, "r") as f:
            self.poses = json.load(f)["poses"]
    
    def apply_pose(self, pose_id=None, pose_name=None):
        if not hasattr(self, "poses"):
            return

        pose = None
        if pose_id is not None:
            pose = next((p for p in self.poses if p["id"] == pose_id), None)
        elif pose_name is not None:
            pose = next((p for p in self.poses if p["name"].lower() == pose_name.lower()), None)

        if pose is None:
            print("Posa non trovata.")
            return

        names = list(pose["angles"].keys())
        angles = list(pose["angles"].values())
        
        self.actual_pose_id = pose["id"]
        self.actual_pose_name = pose["name"]
        self.orientation = pose["orientation"]
        
        if "angles_to_check" in pose:
            self.actual_pose_angles_names = list(pose["angles_to_check"].keys())
            self.actual_pose_angles = list(pose["angles_to_check"].values())
        else:
            self.actual_pose_angles_names = names
            self.actual_pose_angles = angles

        self.motion_service.setAngles(names, angles, 0.2)
        print(f"Applicata posa: {pose['name']}")


def main(args=None):
    rclpy.init(args=args)
    node = NaoTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Chiusura nodo NAO TTS...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
