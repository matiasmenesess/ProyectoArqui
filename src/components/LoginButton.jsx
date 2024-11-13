import React from 'react';
import { Button } from '@mui/material';

const LoginButton = ({ label, icon, hoverColor, onClick, sx }) => {
  return (
    <Button
      fullWidth
      variant="outlined"
      startIcon={icon}
      onClick={onClick}
      sx={{
        ...sx, 
        color: hoverColor,
        borderColor: hoverColor,
        fontFamily: 'Poppins, sans-serif',
        fontWeight: 'bold',
        borderRadius: '50px',
        textTransform: 'none',
        marginTop: 6,
        marginBottom: -4,
        transition: 'background-color 0.3s, color 0.3s',
        '&:hover': {
          backgroundColor: hoverColor,
          color: 'white',
          borderColor: hoverColor,
          '& .MuiSvgIcon-root': {
            color: 'white', 
          },
        },
        '& .MuiSvgIcon-root': {
          color: hoverColor,
        },
      }}
    >
      {label}
    </Button>
  );
};

export default LoginButton;
