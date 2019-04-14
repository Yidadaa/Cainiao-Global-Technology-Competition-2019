import flask

import os
from flask import Flask, request, redirect, url_for, flash
from werkzeug.utils import secure_filename

UPLOAD_FOLDER = './files'
ALLOWED_EXTENSIONS = set(['mp4', 'data'])

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route('/', methods=['GET', 'POST'])
def upload_file():
    if request.method == 'POST':
        if 'file' not in request.files:
            flash('No file part')
            return '0'
        file = request.files['file']
        video = request.files['video']
        # print(request.files)
        for f in [file, video]:
            if f.filename == '':
                flash('No selected file')
                return '0'
            if file and allowed_file(f.filename):
                filename = secure_filename(f.filename)
                f.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
        return '1'

if __name__ == "__main__":
    if not os.path.exists('files'):
        os.mkdir('files')
    app.run(port=2233, host='0.0.0.0', debug=True)