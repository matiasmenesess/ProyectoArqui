import React, { useState } from 'react';
import { Box, Card, CardContent, Typography, TextField, Button, Stack, Link, IconButton, InputAdornment, Alert } from '@mui/material';
import { useNavigate } from 'react-router-dom';
import HeaderLogin from '../components/HeaderLogin';
import Visibility from '@mui/icons-material/Visibility';
import VisibilityOff from '@mui/icons-material/VisibilityOff';

const SignUp = () => {
  const navigate = useNavigate();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [step, setStep] = useState(1);
  const [error, setError] = useState(false);
  const [successAlert, setSuccessAlert] = useState(false);

  const handleNext = () => {
    if (step === 1) {
      // Verificar si el campo de correo electrónico es válido
      if (!email || !/^[A-Z0-9._%+-]+@[A-Z0-9.-]+\.[A-Z]{2,}$/i.test(email)) {
        setError(true);
      } else {
        setError(false);
        setStep(2); // Pasar al siguiente formulario
      }
    }
  };

  const handlePasswordVisibility = () => {
    setShowPassword((prev) => !prev);
  };

  const handleSignUp = () => {
    // Lógica de creación de cuenta (por ahora solo simulación)
    setSuccessAlert(true);
    setTimeout(() => {
      navigate('/login');
    }, 2000); // Redirige al /login después de 2 segundos
  };

  return (
    <Box
      sx={{
        minHeight: '100vh',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        backgroundSize: 'cover',
        backgroundPosition: 'center',
        backgroundRepeat: 'no-repeat',
        backgroundColor: '#f2f4f5',
        padding: 4,
      }}
    >
      <HeaderLogin />

      <Card
        sx={{
          width: '100%',
          maxWidth: 450,
          minHeight: 400,
          padding: 4,
          backgroundColor: 'white',
          borderRadius: 2,
          boxShadow: 4,
          marginTop: '3rem',
          position: 'relative',
        }}
      >
        <CardContent>
          <Stack direction="row" spacing={1} sx={{ marginBottom: 3 }}>
            <Link
              component="button"
              variant="body2"
              onClick={() => navigate('/login')}
              sx={{
                color: '#e31c22',
                textDecoration: 'none',
                cursor: 'pointer',
                fontWeight: 'bold',
                '&:hover': {
                  color: '#b71c1c', // Cambiar a un rojo más oscuro al pasar el mouse
                },
              }}
            >
              Inicia sesión
            </Link>
            <Typography
              variant="body2"
              sx={{
                color: '#1f2937',
              }}
            >
              {'>'}
            </Typography>
            <Typography
              variant="body2"
              sx={{
                color: '#1f2937',
                fontWeight: 'bold',
              }}
            >
              Crear una cuenta
            </Typography>
          </Stack>

          {successAlert && (
            <Alert severity="success" sx={{ marginBottom: 3 }}>
              Cuenta creada correctamente. Redirigiendo a la página de inicio de sesión...
            </Alert>
          )}

          {step === 1 && (
            <>
              <Typography
                variant="h5"
                component="h1"
                align="left"
                sx={{
                  fontFamily: 'Poppins, sans-serif',
                  color: '#1f2937',
                  fontWeight: 600,
                  marginBottom: 3,
                }}
              >
                Crea tu cuenta
              </Typography>

              <Typography
                sx={{
                  fontSize: 16,
                  fontWeight: 'bold',
                  marginBottom: 1,
                }}
              >
                EMAIL
              </Typography>

              <TextField
                fullWidth
                variant="outlined"
                type="email"
                placeholder='nombre@ejemplo.com'
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                error={error}
                helperText={error ? 'Este campo es requerido o el formato es incorrecto' : ''}
                sx={{
                  marginBottom: 4,
                  '& .MuiOutlinedInput-root': {
                    '&.Mui-focused fieldset': {
                      borderWidth: '2px', // Hacer el borde un poco más grueso al enfocarse
                    },
                  },
                  '& .MuiOutlinedInput-root.Mui-error': {
                    '& fieldset': {
                      borderColor: '#e53935',
                      borderWidth: 2,
                    },
                  },
                  '& .MuiInputBase-input': {
                    fontStyle: 'italic',
                    fontSize: '1rem',
                  },
                }}
              />

              <Button
                variant="contained"
                onClick={handleNext}
                sx={{
                  textTransform: 'none',
                  fontFamily: 'Poppins, sans-serif',
                  fontSize: 16,
                  fontWeight: 'bold',
                  backgroundColor: '#e31c22',
                  '&:hover': {
                    backgroundColor: '#b71c1c',
                  },
                  '&:focus': {
                    boxShadow: '0 0 0 4px rgba(0, 0, 0, 0.3)',
                  },
                  width: '100%',
                  padding: '12px 20px',
                  borderRadius: '8px',
                  marginTop: 4,
                }}
              >
                Siguiente
              </Button>
            </>
          )}

          {step === 2 && (
            <>
              <Typography
                variant="h5"
                component="h1"
                align="left"
                sx={{
                  fontFamily: 'Poppins, sans-serif',
                  color: '#1f2937',
                  fontWeight: 600,
                  marginBottom: 3,
                }}
              >
                Crea tu cuenta
              </Typography>

              <Typography
                sx={{
                  fontSize: 16,
                  fontWeight: 'bold',
                  marginBottom: 1,
                }}
              >
                CONTRASEÑA
              </Typography>

              <TextField
                fullWidth
                variant="outlined"
                type={showPassword ? 'text' : 'password'}
                placeholder='Ejemplo123'
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                InputProps={{
                  endAdornment: (
                    <InputAdornment position="end">
                      <IconButton
                        onClick={handlePasswordVisibility}
                        edge="end"
                      >
                        {showPassword ? <VisibilityOff /> : <Visibility />}
                      </IconButton>
                    </InputAdornment>
                  ),
                }}
                sx={{
                  marginBottom: 3,
                  '& .MuiOutlinedInput-root': {
                    '&.Mui-focused fieldset': {
                      borderWidth: '2px',
                    },
                  },
                  '& .MuiInputBase-input': {
                    fontStyle: 'italic',
                    fontSize: '1rem',
                  },
                }}
              />

              <Typography variant="body2" sx={{ marginBottom: 3 }}>
                ✔ 8 caracteres &nbsp; ✔ Una mayúscula &nbsp; ✔ Una minúscula &nbsp; ✔ Un número
              </Typography>

              <Typography variant="body2" sx={{ marginBottom: 3 }}>
                Al crear una cuenta aceptas:
                <Link href="#" sx={{ color: '#3b82f6', textDecoration: 'underline' }}> nuestra política de privacidad</Link>.<br />
                *Recibir ofertas y recomendaciones por email
              </Typography>

              <Button
                variant="contained"
                onClick={handleSignUp}
                sx={{
                  textTransform: 'none',
                  fontFamily: 'Poppins, sans-serif',
                  fontSize: 16,
                  fontWeight: 'bold',
                  backgroundColor: '#e31c22',
                  '&:hover': {
                    backgroundColor: '#b71c1c',
                  },
                  width: '100%',
                  padding: '12px 20px',
                  borderRadius: '8px',
                }}
              >
                Crear una cuenta
              </Button>
            </>
          )}
        </CardContent>
      </Card>
    </Box>
  );
};

export default SignUp;
