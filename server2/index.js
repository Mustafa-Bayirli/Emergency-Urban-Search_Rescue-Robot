const express = require('express');
const app = express();
const bodyParser = require('body-parser');
const fs = require('fs');
const path = require('path');
const cors = require('cors');

app.use(cors());
app.use(express.json({ limit: '100mb' }));
app.use(express.urlencoded({ extended: true, limit: '100mb' }));
app.use(express.json())
const port = 5000;

app.use(bodyParser.json());


//Post for screenshot
app.post('/save-screenshot', (req, res) => {
  const { image } = req.body;

  try {
    const fileName = `screenshot_${Date.now()}.png`;
    const buffer = Buffer.from(image.replace(/^data:image\/\w+;base64,/, ''), 'base64');
    fs.writeFileSync(path.join(__dirname, 'screenshots', fileName), buffer);

    console.log(`File saved: ${fileName}`);

    res.json({ fileName });
  } catch (error) {

    console.error(error);
    res.status(500).json({ error: 'Internal Server Error' });
  }
});


app.listen(port, () => {
    console.log(`Server is running on port ${port}`);
  });