# การโปรแกรมไพทอนสำหรับงานควบคุมและฝังตัว

## Python Programming for Control and Embedded Applications

<img src="https://drive.google.com/uc?id=19irK_Lqk0BtMGrlq-kmb6powwAsoIg0Q" width=600 alt="py4conemb_cover"/>
<hr>
<br><b>ผู้เขียน :</b> ดร. วโรดม ตู้จินดา
<br>ภาควิชาวิศวกรรมเครื่องกล คณะวิศวกรรมศาสตร์
<br>มหาวิทยาลัยเกษตรศาสตร์
<p />
<p />
<b>ISBN :</b> 978-616-608-619-5
<br><b>จำนวนหน้า :</b> 460 
<p />

**5 กค. 2567 :** หนังสือได้รับรางวัลตำราดีเด่นจากคณะวิศวกรรมศาสตร์ ม.เกษตรศาสตร์
<img src="https://raw.githubusercontent.com/dewdotninja/sharing-github/master/py4conemb_cert.jpg" width=200 />
  
หนังสือเล่มนี้ไม่มีจำหน่ายเป็นเล่มพิมพ์ แต่สามารถดาวน์โหลด PDF ได้ฟรีทั้่งเล่ม ไม่ต้องไปหาที่อื่นไกล คลิกลิงก์นี้เลย 
<br><a href="https://drive.google.com/file/d/1NplIDw-kojc8b0gLTpcQZE3m9YRQV7M3/view?usp=sharing">py4conemb</a>

<hr>
<p />
ไพทอนเป็นภาษาคอมพิวเตอร์ที่มีเสน่ห์ในตัวเอง เช่นความเป็นเชิงวัตถุ มีความยืดหยุ่นสูง หาไลบรารี่สนับสนุนได้ง่าย และไวยากรณ์ภาษาที่ช่วยให้เขียนโค้ดได้กระชับ ส่วนทางด้านสมรรถนะการคำนวณเมื่อประมวลผลบนคอมพิวเตอร์สมรรถนะสูงเทียบเคียงได้กับผลิตภัณฑ์ซอฟต์แวร์อื่นที่ใช้งานด้านวิศวกรรม ส่วนข้อเสียเปรียบก็จะเหมือนกับภาษาระดับสูงทั่วไป คือขนาดของโค้ดและความเร็วในการประมวลผลโดยทั่วไปยังด้อยกว่าโค้ดที่เขียนภาษาระดับกลางและผ่านการคอมไพล์เช่นภาษา C++ อยู่บ้าง โดยเฉพาะในระบบฝังตัวที่มีข้อจำกัดด้านทรัพยากร แต่ในปัจจุบันเมื่อพิจารณาในภาพรวมกับสมรรถนะของคอมพิวเตอร์และไมโครคอนโทรลเลอร์ กล่าวได้ว่าเป็นแนวทางการพัฒนาที่น่าสนใจจนต้องรวบรวมเนื้อหาไว้ใช้ประโยชน์สำหรับผู้เริ่มต้นจนถึงขั้นสูง
<p />
หนังสือ “การโปรแกรมไพทอนสำหรับงานควบคุมและฝังตัว” เรียบเรียงขึ้นเพื่อใช้ประกอบการสอนในรายวิชาของสาขาวิศวกรรมไฟฟ้าเครื่องกลการผลิต ระดับปริญญาตรี และหลักสูตรวิศวกรรมศาสตรมหาบัณฑิต สาขาเทคโนโลยีการผลิตทางอุตสาหกรรม คณะวิศวกรรมศาสตร์ มหาวิทยาลัยเกษตรศาสตร์ หรือใช้อ่านประกอบสำหรับผู้สนใจทั่วไป เนื้อหาหลักเน้นการประยุกต์ใช้งานด้านวิศวกรรมควบคุม การเขียนโปรแกรมโดยใช้แพ็กเกจสนับสนุน ตั้งแต่พื้นฐานจนถึงเนื้อหาขั้นสูงเช่นการโปรแกรมเชิงวัตถุ รวมถึงการพัฒนาสำหรับระบบฝังตัวโดยใช้ไมโครไพทอน (micropython)  ซึ่งกล่าวได้ว่าเป็นเซตย่อยของภาษาไพทอนสำหรับไมโครคอนโทรลเลอร์ เนื้อหาในส่วนนี้ครอบคลุมถึงระบบฝังตัวที่ได้รับความนิยมในปัจจุบันเช่นไอโอทีและหุ่นยนต์ 

<hr>

### แก้ไข/เพิ่มเติม/อัพเดต

#### 14 พค. 67

เพิ่มโค้ดในส่วนของการใช้ virtual serial port สื่อสารระหว่าง Wokwi กับ tkinter บน 
Jupyter notebook โดยรวบรวมไว้ในโฟลเดอร์ vsp 

#### 30 เมย. 67

แก้ไขจุดผิดพลาดในโค้ด Wokwi สำหรับการควบคุมมอเตอร์ข้อต่อเดี่ยว ในส่วนของ dcm.chip.c และในไฟล์ main.py ที่ทำให้การควบคุมป้อนกลับสถานะทำงานไม่ถูกต้อง 
จุดที่ผิดคือการนิยามขาเอาต์พุตบนชิปเฉพาะของ Wokwi และต้องเพิม offset ให้กับการอ่านค่าความเร็วจากชิป หลังแก้ไขแล้วการออกแบบโดยวิธีวางโพลใช้งานได้ดี 
แต่การจำกัดค่าเอาต์พุตตัวควบคุมทำให้ไม่สามารถกำหนดค่าความถี่ $\omega_n$ ได้สูงมากเท่าการจำลองบน Jupyter notebook


#### 20 กพ. 67

ไลบรารี paho-mqtt เวอร์ชันใหม่ที่ 2.0.0 ที่เริ่มใช้วันที่ 11 กพ. 67 มีการเปลี่ยนแปลงโค้ดสำหรับตั้งค่าเริ่มต้น client ทำให้โค้ดที่อยู่ใน notebook บทที่ 8 ตรงส่วนนี้

```python
Client_ID = ""
User = ""
Password = ""
client = mqtt.Client(client_id=Client_ID,
                         transport='tcp',
                         protocol=mqtt.MQTTv311,
                         clean_session=True)
client.username_pw_set(User,Password)
```
ไม่สามารถทำงานได้ ต้องมีการแก้ไขเป็นดังนี้

```python
Client_ID = ""
User = ""
Password = ""
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id=Client_ID,
                         transport='tcp',
                         protocol=mqtt.MQTTv311,
                         clean_session=True)
client.username_pw_set(User,Password)
```

จะเห็นว่ามีการเพิ่มอาร์กิวเมนต์แรกคือ mqtt.CallbackAPIVersion.VERSION2 ซึ่งจำเป็นต้องมี จะเพิ่มโค้ดส่วนที่แก้ไขแล้วอยู๋ในไดเรคทอรีย่อย /v2/

<hr>

### โค้ดไพทอน

<b>Disclaimer : </b>โค้ดไพทอนที่รวบรวมไว้ใน github repo นี้ รวมถึงส่วนที่พัฒนาบนแพลตฟอร์ม Wokwi ตามลิงก์ด้านล่าง สามารถนำไปประยุกต์/ใช้ประโยชน์ได้อย่างเสรีโดยไม่มีลิขสิทธิ์ใดๆ แต่เนื่องจากวัตถุประสงค์หลักคือใช้ประกอบตัวอย่างในหนังสือ ผู้เขียนได้ตรวจสอบเพึยงใช้งานไ้ด้ถูกต้องตามวัตถุประสงค์สำหรับตัวอย่างนั้น แต่ไม่สามารถรับประกันว่าปลอดจากจุดบกพร่อง (bugs) โดยสมบูรณ์สำหรับการใช้ในงานจริง โดยเฉพาะในระบบฝังตัวที่จุดบกพร่องอาจขึ้นกับหลายปัจจัยรวมถึงฮาร์ดแวร์ โดยเหตุนี้ผู้เขียนต้องขอปฏิเสธความรับผิดชอบในทุกกรณีหากเกิดอันตราย/ ความเสียหายต่อชีวิตและทรัพย์สินจากการประยุกต์ใช้งานซอฟต์แวร์ประกอบหนังสือ

#### ลิงก์โพรเจกบน Wokwi

<b>หมายเหตุ : </b> บางโพรเจกมิได้ระบุในหนังสือโดยตรง แต่เกี่ยวข้องกับเนื้อหาในบทนั้น

<ul>
<li />บทที่ 4
<ul>
<li /><a href="https://wokwi.com/projects/387226918295246849">การกระพริบ LED</a> คือโปรแกรม "Hello world" ของระบบฝังตัว
<li />ตัวอย่าง 4.9 : <a href="https://wokwi.com/projects/387227633700996097">cmdtest2.py</a>

</ul>

<li />บทที่ 5
<ul>
<li />ตัวอย่าง 5.5 : <a href="https://wokwi.com/projects/387228145560282113">iot_controller4.py</a> (อย่าลืมใส่ข้อมูลจาก NETPIE 2020 ก่อนรัน)

</ul>

<li />บทที่ 6
์<ul>
<li />ตัวอย่าง 6.5 : <a href="https://wokwi.com/projects/387227152163383297">rgbfancy.py</a>
<li /><a href="https://wokwi.com/projects/387227281091069953">rgbfancy_netpie.py</a> แสดงการเสริมโปรแกรม rgbfancy.py ให้ใช้งานไอโอทีได้
</ul>
<li />บทที่ 8
<ul>
<li />ตัวอย่าง 8.1 : <a href="https://wokwi.com/projects/387228486819383297">dcm1_controllers.py</a> แสดงการควบคุมข้อต่อหุ่นยนต์ที่ขับเคลื่อนโดยดีซีมอเตอร์ ใช้สัญญาณ PWM/DIR เป็นเอาต์พุตตัวควบคุม 
<li />ตัวอย่าง 8.2 - 8.6 : การควบคุมแบบไม่เป็นเชิงเส้นสำหรับแขนกล 2 ก้านต่อ รวมอยู่ในโพรเจกนี้ <a href="https://wokwi.com/projects/387228698016260097">robo2links_track.py</a>
<li />ตัวอย่าง 8.7 - 8.8 : <a href="https://wokwi.com/projects/387229048610888705">robo2links_iot.py</a> ใช้ร่วมกับ GUI ในไฟล์ chapter8.ipynb เพื่อศึกษาจลนศาสตร์ของแขนกล 2 ก้านต่อ
<li />ตัวอย่าง 8.9 : <a href="https://wokwi.com/projects/387229353821011969">ddrobot_controllers.py</a>ตัวควบคุมหุ่นยนต์ส้ม เพื่อศึกษาการควบคุมขั้นพื้นฐาน โดยยังไม่มีการเชื่อมต่อไอโอที 
<li />ตัวอย่าง 8.10 : <a href="https://wokwi.com/projects/387229438026931201">ddrobot_controllers_iot.py</a> เสริมไอโอทีให้หุ่นยนต์ส้ม ใช้ร่วมกับ GUI ในไฟล์ ddtk1.ipynb ที่สามารถกำหนดตำแหน่งเป้าหมายโดยใช้เม้าส์ และให้หุ่นยนต์เคลื่อนที่สู่เป้าหมายนั้น
<li />ตัวอย่าง 8.11 : การเคลื่อนที่ของหุ่นยนต์ส้มในกริดเวิลด์ ใช้ <a href="https://wokwi.com/projects/387229597666359297">ddrobot_iot_gw.py</a> ร่วมกับ GUI ในไฟล์ ddtk2.ipynb เพื่อกำหนดสภาพแวดล้อมของกริดเวิลด์ และหาคำตอบของการวนซ้ำมูลค่า เพื่อได้นโยบายเหมาะที่สุดสำหรับเส้นทางที่หุ่นยนต์จะเคลื่อนที่
</ul>
<li />เสริม : ใช้พอร์ตอนุกรมเสมือน (virtual serial port) แทน NETPIE IoT
<ul>
<li /><a href="https://wokwi.com/projects/396312701017628673">จำลองดีซีมอเตอร์</a>
<li /><a href="https://wokwi.com/projects/396935752500927489">แขนกล 2 ก้านต่อ</a>
<li /><a href="https://wokwi.com/projects/397298628049688577">หุ่นยนต์ส้มเคลื่อนที่อิสระ</a>
<li /><a href="https://wokwi.com/projects/397732021696556033">หุ่นยนต์ส้มเคลื่อนที่บนกริดเวิลด์</a>
</ul>

</ul>

### วีดีโอ

คลิปวีดีโอคัดเลือกที่เกี่ยวข้องกับเนื้อหาในหนังสือ คลิปอื่นดูได้จากช่อง <a href="https://www.youtube.com/@varodomt">@varodomt</a>

<ul>
<li />บทที่ 2
<ul>
<li /><a href="https://youtu.be/ga2dXXfAiFk?si=HSCCF3p7DPQcigoQ">วีดีโอ 03 (220533) : วงจรไฟฟ้ากระแสสลับและสัญญาณที่เป็นฟังก์ชันของเวลา</a>
</ul>

<li />บทที่ 3
<ul>
<li /><a href="https://youtu.be/24eO3mYsSaA?si=O7WLQB4Ee5xJaiu9">Classical Loop Shaping (Thai)</a>
</ul>

<li />บทที่ 5
<ul>
<li /><a href="https://youtu.be/XHzSaWOAS0E?si=JPH8nAf90NuZly6s">NETPIE2020 dashboard : การส่ง message จาก ESP32 ไปยัง dashboard เพื่ออัพเดต</a>
</ul>
<li />บทที่ 6
<ul>
<li /><a href="https://youtu.be/a0tH8vJ3T0g?si=tzwP95jq2MsYPfZP">จำลอง micropython สำหรับ ESP32 และ NETPIE2020 บนเว็บ Wokwi</a>
</ul>
<li />บทที่ 8
<ul>
<li /><a href="https://youtu.be/3to1_SIWH64?si=RJJlr-ashozVOH8Z">จำลองพลวัตแขนกล 2 ก้านต่อเป็น custom chip บน wokwi</a>
<li /><a href="https://youtu.be/P5REf6HBmz0?si=14ktX0IOTdaUbArB">ใช้ NETPIE IoT จำลองจลนศาสตร์แขนกลบน Jupyter notebook</a>
<li /><a href="https://youtu.be/Dfr7qgxvRS8?si=6LlR_nIbKrHQhWb5">ตัวควบคุมไอโอทีสำหรับหุ่นยนต์สองล้อ (differential drive robot)</a>
<li /><a href="https://youtu.be/3vH49engcdo?si=U-qlE9JvXggqxU1C">แก้ปัญหากริดเวิลด์เพื่อวางแผนเส้นทางเดินหุ่นยนต์</a>
</ul>
<li />ภาคผนวก A
<ul>
<li /><a href="https://youtu.be/lH7kSj0lai0?si=KllavgeFK3-qPKYj">ทดลองใช้ Paho MQTT library สื่อสารกับอุปกรณ์ IoT ผ่าน NETPIE 2020</a>
</ul>

<li />ภาคผนวก B
<ul>
<li /><a href="https://youtu.be/-P7gBgtUN0w?si=pzosqkCR48jrDkw4">สร้าง custom chip บน Wokwi เพื่อจำลองพลวัตของพลานต์</a>
</ul>
</ul>

### วีดีโอเสริม

<ul>
<li />การใช้พอร์ตอนุกรมเสมือน (virtual serial port) แทน IoT
<ul>
<li /><a href="https://youtu.be/7BJiWQn4hBs">รับส่งข้อมูลระหว่าง Wokwi กับ Jupyter notebook ผ่าน virtual serial port</a>
<li /><a href="https://youtu.be/77J155Bu_eI">ทดลองใช้ Virtual serial port กับการจำลองหุ่นยนต์</a>
</ul>
</ul>

<b>หมายเหตุ : </b>
หน้าตาของเครื่องมือซอฟต์แวร์ที่ใช้อาจมีการเปลี่ยนแปลงบ้างเมื่อมีการอัพเดตเวอร์ชัน 
