FROM python:3.10.13

COPY requirements.txt ./
RUN pip install -r requirements.txt

COPY . .

CMD [ "python", "./BlueSky.py", "--headless", "--configfile","settings_c2c.cfg" , "--scenfile","c2c/test_c2c.scn"]