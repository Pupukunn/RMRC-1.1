import React, { useState } from 'react';

function ToggleButton() {
  const [modal, setmodal] = useState(false);

  const togglemodal = () => {
    setmodal(!modal)
  }

  return(
    <>
    <button onClick={togglemodal}></button>
    </>
  )
}
export default ToggleButton;